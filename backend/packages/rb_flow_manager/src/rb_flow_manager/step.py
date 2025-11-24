import uuid
from collections.abc import Callable
from multiprocessing import Event
from typing import Any

from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import call_with_matching_args, safe_eval_expr

from .context import ExecutionContext
from .exception import StopExecution
from .utils import _resolve_arg_scope_value


class Step:
    """실행 단계 (children을 가질 수 있음)"""

    def __init__(
        self,
        step_id: str,
        name: str,
        variable: dict[str, Any] | None = None,
        disabled: bool | None = None,
        func: Callable | None = None,
        repeat_count: int | str | None = None,
        done_script: Callable | str | None = None,
        children: list["Step"] | None = None,
        *,
        func_name: str | None = None,
        args: dict[str, Any] | None = None,
    ):
        self._class_name = "Step"
        self.step_id = step_id
        self.name = name
        self.variable = variable
        self.disabled = disabled
        self.func_name = func_name
        self.done_script = done_script
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
            variable=d.get("variable"),
            func_name=d.get("funcName"),
            disabled=d.get("disabled"),
            args=d.get("args") or {},
            done_script=d.get("doneScript"),
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "disabled": self.disabled,
            "variable": self.variable,
            "funcName": self.func_name,
            "args": self.args,
            "method": "Step",
            "children": [child.to_dict() for child in self.children],
        }

    def to_py_string(self, depth: int = 0):
        def _indent(s: str, n: int) -> str:
            pad = " " * n
            return "\n".join(pad + line if line else line for line in s.splitlines())

        def format_dict(d: dict, indent: int) -> str:
            pad = " " * 4
            items = []
            for k, v in d.items():
                items.append(f"{pad}{repr(k)}: {repr(v)},")
            return "{\n" + "\n".join(items) + "\n" + " " + "}"

        if self.children:
            inner_steps = [child.to_py_string(depth + 8) for child in self.children]
            inner_src = ",\n".join(inner_steps)

            children_block = "[\n" + inner_src + "\n" + (" " * (depth + 4)) + "]"
        else:
            children_block = "[]"

        variable_block = ""
        if self.variable:
            v = format_dict(self.variable, depth)
            variable_block = _indent(f"variable={v},\n\n", depth + 4)

        # args block
        args_block = ""
        if self.args:
            a = format_dict(self.args, depth + 4)
            args_block = _indent(f"args={a},\n\n", depth + 4)

        return (
            (" " * depth)
            + f"{self._class_name}(\n"
            + _indent(f"step_id={repr(self.step_id)},", depth + 4)
            + "\n"
            + variable_block
            + _indent(f"name={repr(self.name)},", depth + 4)
            + "\n"
            + (_indent(f"disabled={repr(self.disabled)},\n\n", depth + 4) if self.disabled else "")
            + (
                _indent(f"func_name={repr(self.func_name)},\n\n", depth + 4)
                if self.func_name
                else ""
            )
            + args_block
            + (
                _indent(f"done_script={repr(self.done_script)},\n", depth + 4)
                if self.done_script is not None
                else ""
            )
            + (" " * (depth + 4))
            + f"children={children_block},\n"
            + (" " * depth)
            + ")"
        )

    def execute(self, ctx: ExecutionContext):
        """단계 실행"""

        done_called = False

        ctx.push_args(self.args)
        ctx.emit_next(self.step_id)
        ctx.check_stop()

        try:
            fn = self.func
            func_name = _resolve_arg_scope_value(self.func_name, ctx) if self.func_name else None
            args = _resolve_arg_scope_value(self.args, ctx) if self.args else {}

            if fn is None and func_name and not self.disabled:
                fn = ctx.sdk_functions.get(func_name)
                if fn is None:
                    raise StopExecution(f"unknown flow function: {func_name}")

            if fn is not None and not self.disabled:
                done_event = Event()

                def done():
                    nonlocal done_called

                    if done_called:
                        return

                    done_called = True

                    if self.done_script is not None:
                        if isinstance(self.done_script, str):
                            safe_eval_expr(
                                self.done_script,
                                variables=ctx.variables,
                                get_global_variable=ctx.get_global_variable,
                            )
                        else:
                            self.done_script()

                    done_event.set()

                flow_manager_args = FlowManagerArgs(ctx=ctx, args=self.args, done=done)

                try:
                    eval_args = {}

                    for k, v in args.items():
                        eval_args[k] = safe_eval_expr(
                            v, variables=ctx.variables, get_global_variable=ctx.get_global_variable
                        )

                    if self.variable:
                        for k, v in self.variable.items():
                            if isinstance(v, str):
                                if v.startswith("$parent."):
                                    attr = v[len("$parent.") :]
                                    v = ctx.lookup(attr)
                                elif v.startswith("$args."):
                                    attr = v[len("$args.") :]
                                    v = eval_args.get(attr)
                                else:
                                    v = safe_eval_expr(
                                        v,
                                        variables=ctx.variables,
                                        get_global_variable=ctx.get_global_variable,
                                    )

                            self.variable[k] = v

                        ctx.update_variables(self.variable)

                    call_with_matching_args(fn, **eval_args, flow_manager_args=flow_manager_args)
                except RuntimeError as e:
                    ctx.emit_error(self.step_id, RuntimeError(str(e)))
                    ctx.stop()
                    raise StopExecution(str(e)) from e
                except Exception as e:  # noqa: BLE001
                    ctx.emit_error(self.step_id, e)
                    print(f"[{ctx.process_id}] Step '{self.name}' error: {e}")
                    ctx.stop()
                    raise StopExecution(str(e)) from e

                while not done_event.wait(timeout=0.01):
                    ctx.check_stop()

            ctx.emit_done(self.step_id)

            # 자식 노드들을 순차 실행
            for child in self.children:
                ctx.check_stop()
                child.execute(ctx)
        finally:
            ctx.pop_args()


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
