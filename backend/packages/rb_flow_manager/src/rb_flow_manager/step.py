from collections.abc import Callable
from multiprocessing import Event

from rb_schemas.sdk import FlowManagerArgs

from rb_flow_manager.utils import call_with_matching_args, safe_eval_expr

from .context import ExecutionContext
from .exception import StopExecution


class Step:
    """실행 단계 (children을 가질 수 있음)"""

    def __init__(
        self,
        step_id: str,
        name: str,
        func: Callable | None = None,
        children: list | None = None,
        *,
        func_name: str | None = None,
        args: dict[str, str] | None = None,
    ):
        self.step_id = step_id
        self.name = name
        self.func_name = func_name
        self.children = children or []
        self.args = args or {}

        self.func = func

    def execute(self, ctx: ExecutionContext):
        """단계 실행"""
        ctx.check_stop()

        # func가 있으면 실행
        fn = self.func
        if fn is None and self.func_name:
            fn = ctx.sdk_functions.get(self.func_name)
            if fn is None:
                raise StopExecution(f"unknown flow function: {self.func_name}")

        if fn is not None:
            done_event = Event()

            def done():
                ctx.emit_next(self.step_id)
                done_event.set()

            flow_manager_args = FlowManagerArgs(ctx=ctx, done=done)

            try:
                eval_args = {
                    k: safe_eval_expr(v, variables=ctx.variables) for k, v in self.args.items()
                }
                call_with_matching_args(fn, **eval_args, flow_manager_args=flow_manager_args)
            except RuntimeError as e:
                ctx.stop()
                ctx.emit_error(self.step_id, e)
                raise StopExecution(str(e)) from e
            except Exception as e:  # noqa: BLE001
                print(f"[{ctx.process_id}] Step '{self.name}' error: {e}")
                ctx.stop()
                ctx.emit_error(self.step_id, e)
                raise StopExecution(str(e)) from e

            while not done_event.wait(timeout=0.01):
                ctx.check_stop()

        # 자식 노드들을 순차 실행
        for child in self.children:
            ctx.check_stop()
            child.execute(ctx)


class RepeatStep(Step):
    """반복 Step"""

    def __init__(self, step_id: str, name: str, count: int, children: list | None = None):
        super().__init__(step_id, name, func=None, children=children)
        self.count = count

    def execute(self, ctx: ExecutionContext):
        """지정된 횟수만큼 자식 Step 반복 실행"""
        for i in range(self.count):
            ctx.check_stop()
            ctx.data["repeat_index"] = i
            for child in self.children:
                ctx.check_stop()
                child.execute(ctx)


class ConditionStep(Step):
    """조건 Step"""

    def __init__(self, step_id: str, name: str, condition: Callable, children: list | None = None):
        super().__init__(step_id, name, func=None, children=children)
        self.condition = condition

    def execute(self, ctx: ExecutionContext):
        """조건이 참일 때만 자식 Step 실행"""
        ctx.check_stop()
        if self.condition(ctx):
            for child in self.children:
                ctx.check_stop()
                child.execute(ctx)
