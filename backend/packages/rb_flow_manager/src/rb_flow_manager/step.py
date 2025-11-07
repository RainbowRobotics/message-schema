import uuid
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
        children: list["Step"] | None = None,
        *,
        func_name: str | None = None,
        args: dict[str, str] | None = None,
    ):
        self._class_name = "Step"
        self.step_id = step_id
        self.name = name
        self.func_name = func_name
        self.children = children or []
        self.args = args or {}

        self.func = func

    @staticmethod
    def from_dict(d) -> "Step":
        if d.get("method") == "Repeat":
            return RepeatStep.from_dict(d)

        return Step(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            func_name=d.get("funcName"),
            args=d.get("args") or {},
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "funcName": self.func_name,
            "args": self.args,
            "method": "Step",
            "children": [child.to_dict() for child in self.children],
        }

    def to_py_string(self, depth: int = 0):
        def _indent(s: str, n: int) -> str:
            pad = " " * n
            return "\n".join(pad + line if line else line for line in s.splitlines())

        if self.children:
            inner_steps = [child.to_py_string(depth + 8) for child in self.children]
            inner_src = ",\n".join(inner_steps)

            children_block = "[\n" + inner_src + "\n" + (" " * (depth + 4)) + "]"
        else:
            children_block = "[]"

        return (
            (" " * depth)
            + f"{self._class_name}(\n"
            + _indent(f"step_id={repr(self.step_id)},", depth + 4)
            + "\n"
            + _indent(f"name={repr(self.name)},", depth + 4)
            + "\n"
            + _indent(f"func_name={repr(self.func_name)},", depth + 4)
            + "\n"
            + (" " * (depth + 4))
            + f"children={children_block},\n"
            + _indent(f"args={repr(self.args)},", depth + 4)
            + "\n"
            + (" " * depth)
            + ")"
        )

    def execute(self, ctx: ExecutionContext):
        """단계 실행"""
        ctx.emit_next(self.step_id)
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
                ctx.emit_complete(self.step_id)
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
        self._class_name = "RepeatStep"

    @staticmethod
    def from_dict(d) -> "RepeatStep":
        return RepeatStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            count=int(d.get("count", 1)),
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "method": "Repeat",
            "count": self.count,
            "children": [child.to_dict() for child in self.children],
        }

    def to_py_string(self, depth: int = 0):
        def _indent(s: str, n: int) -> str:
            pad = " " * n
            return "\n".join(pad + line if line else line for line in s.splitlines())

        if self.children:
            inner_steps = [child.to_py_string(depth + 8) for child in self.children]
            inner_src = ",\n".join(inner_steps)

            children_block = "[\n" + inner_src + "\n" + (" " * (depth + 4)) + "]"
        else:
            children_block = "[]"

        return (
            (" " * depth)
            + f"{self._class_name}(\n"
            + _indent(f"step_id={repr(self.step_id)},", depth + 4)
            + "\n"
            + _indent(f"name={repr(self.name)},", depth + 4)
            + "\n"
            + _indent(f"count={self.count},", depth + 4)
            + "\n"
            + (" " * (depth + 4))
            + f"children={children_block},\n"
            + _indent(f"args={repr(self.args)},", depth + 4)
            + "\n"
            + (" " * depth)
            + ")"
        )

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
