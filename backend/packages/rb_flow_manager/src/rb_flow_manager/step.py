import time
import uuid
from collections.abc import Callable
from multiprocessing import Event
from typing import Any

from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import call_with_matching_args, eval_value, safe_eval_expr

from .context import ExecutionContext
from .exception import BreakFolder, StopExecution
from .utils import _resolve_arg_scope_value


class Step:
    """실행 단계 (children을 가질 수 있음)"""

    def __init__(
        self,
        *,
        step_id: str,
        name: str,
        variable: dict[str, Any] | None = None,
        disabled: bool | None = None,
        method: str | None = None,
        func: Callable | None = None,
        done_script: Callable | str | None = None,
        children: list["Step"] | None = None,
        func_name: str | None = None,
        args: dict[str, Any] | None = None,
        **kwargs,
    ):
        self._class_name = getattr(self, "_class_name", "Step")
        self._start_ts: float = 0
        self.step_id = step_id
        self.name = name
        self.variable = variable
        self.disabled = disabled
        self.method = method
        self.func_name = func_name
        self.done_script = done_script
        self.children = children or []
        self.args = args or {}
        self.kwargs = kwargs
        self.func = func

    @staticmethod
    def from_dict(d) -> "Step":
        if d.get("method") == "Folder":
            return FolderStep.from_dict(d)

        if d.get("method") == "Repeat":
            return RepeatStep.from_dict(d)

        if d.get("method") in ["If", "ElseIf", "Else", "Condition"]:
            return ConditionStep.from_dict(d)

        return Step(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            memo=d.get("memo"),
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

        other_kwargs_block = ""
        if self.kwargs:
            for k, v in self.kwargs.items():
                other_kwargs_block += (
                    _indent(f"{k}={repr(v)},\n\n", depth + 4) if v is not None else ""
                )

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
            + other_kwargs_block
            + (" " * (depth + 4))
            + f"children={children_block},\n"
            + (" " * depth)
            + ")"
        )

    def _pre_execute(self, ctx: ExecutionContext, *, reset_condition_result: bool = True):
        if reset_condition_result:
            condition_map: dict[str, bool] = ctx.data.get("condition_map", {})

            for key in condition_map:
                if key.endswith(f".{ctx.current_depth}"):
                    condition_map.pop(key)
            ctx.data["condition_map"] = condition_map

        ctx.push_args(self.args)

        ctx.step_barrier(self.step_id)

        self._start_ts = time.monotonic()

    def _post_execute(self, ctx: ExecutionContext):
        min_step_interval = getattr(ctx, "min_step_interval", 0)
        elapsed = time.monotonic() - self._start_ts
        remaining = min_step_interval - elapsed

        if remaining > 0 and ctx.step_num != 1:
            time.sleep(remaining)

        ctx.step_num += 1

    def execute_children(self, ctx: ExecutionContext):
        """자식 Step들을 순차적으로 실행"""

        if len(self.children) > 0:
            ctx.current_depth += 1

        for child in self.children:
            try:
                child.execute(ctx)
            except BreakFolder:
                if getattr(self, "method", None) == "Folder":
                    break
                raise
            finally:
                ctx.pop_args()

        if len(self.children) > 0:
            ctx.current_depth -= 1

    def execute(self, ctx: ExecutionContext):
        """단계 실행"""
        self._pre_execute(ctx)

        ctx.emit_next(self.step_id)

        done_called = False

        ctx.check_stop()


        fn = self.func
        func_name = _resolve_arg_scope_value(self.func_name, ctx) if self.func_name else None
        args = _resolve_arg_scope_value(self.args, ctx) if self.args else {}

        if fn is None and func_name and not self.disabled:
            fn = ctx.sdk_functions.get(func_name) if ctx.sdk_functions is not None else None
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
                    eval_args[k] = eval_value(
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
                                v = eval_value(
                                    v,
                                    variables=ctx.variables,
                                    get_global_variable=ctx.get_global_variable,
                                )

                        self.variable[k] = v

                    ctx.update_local_variables(self.variable)

                    if "robot_model" not in eval_args:
                        eval_args["robot_model"] = ctx.state_dict.get("robot_model", None)

                call_with_matching_args(fn, **eval_args, flow_manager_args=flow_manager_args)
            except RuntimeError as e:
                ctx.emit_error(self.step_id, RuntimeError(str(e)))
                print(f"[{ctx.process_id}] Step '{self.name}' error: {e}", flush=True)
                time.sleep(0.1)
                ctx.stop()
                raise StopExecution(str(e)) from e
            except Exception as e:  # noqa: BLE001
                ctx.emit_error(self.step_id, e)
                print(f"[{ctx.process_id}] Step '{self.name}' error: {e}", flush=True)
                time.sleep(0.1)
                ctx.stop()
                raise StopExecution(str(e)) from e

            while not done_event.wait(timeout=0.01):
                ctx.check_stop()

        self._post_execute(ctx)

        ctx.emit_done(self.step_id)

        self.execute_children(ctx)



class FolderStep(Step):
    """폴더 Step"""

    _class_name = "FolderStep"

    def __init__(
        self,
        *,
        step_id: str,
        name: str,
        func_name: str | None = None,
        func: Callable | None = None,
        children: list | None = None,
        args: dict[str, Any] | None = None,
    ):
        super().__init__(
            step_id=step_id,
            name=name,
            method="Folder",
            func_name=func_name,
            func=func,
            children=children,
            args=args,
        )
        self.children = children or []

    @staticmethod
    def from_dict(d) -> "FolderStep":
        return FolderStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            func_name=d.get("funcName"),
            func=d.get("func"),
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
            args=d.get("args") or {},
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "funcName": self.func_name,
            "func": self.func,
            "method": self.method,
            "children": [child.to_dict() for child in self.children],
            "args": self.args,
        }

    def execute(self, ctx: ExecutionContext):
        ctx.enter_folder()

        try:
            super().execute(ctx)
        finally:
            ctx.leave_folder()

class RepeatStep(Step):
    """반복 Step"""

    _class_name = "RepeatStep"

    def __init__(
        self,
        *,
        step_id: str,
        name: str,
        count: int,
        while_cond: str | Callable | None = None,
        do_while: str | Callable | None = None,
        args: dict[str, Any] | None = None,
        children: list | None = None,
    ):
        super().__init__(
            step_id=step_id,
            name=name,
            func=None,
            count=count,
            while_cond=while_cond,
            do_while=do_while,
            args=args,
            children=children,
        )
        self.count = count
        self.while_cond = while_cond
        self.do_while = do_while
        self.args = args or {}
        self.children = children or []

    @staticmethod
    def from_dict(d) -> "RepeatStep":
        count = d.get("count") or d.get("args", {}).get("count", 1)
        while_cond = d.get("whileCond") or d.get("args", {}).get("while_cond")
        do_while = d.get("doWhile") or d.get("args", {}).get("do_while")

        if count is None and while_cond is None and do_while is None:
            raise RuntimeError("count, while_cond, or do_while is required")

        return RepeatStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            count=count,
            while_cond=while_cond,
            do_while=do_while,
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "count": self.count,
            "whileCond": self.while_cond,
            "doWhile": self.do_while,
            "children": [child.to_dict() for child in self.children],
            "args": {
                "count": self.count,
                "while_cond": self.while_cond,
                "do_while": self.do_while,
            },
        }

    def execute(self, ctx: ExecutionContext):
        """지정된 횟수만큼 자식 Step 반복 실행"""
        self._pre_execute(ctx)

        ctx.emit_next(self.step_id)

        ctx.check_stop()

        if self.count is not None:
            for i in range(self.count):
                ctx.check_stop()
                ctx.data["repeat_index"] = i
                self.execute_children(ctx)

        elif self.while_cond is not None:
            if isinstance(self.while_cond, str):
                while safe_eval_expr(
                    self.while_cond,
                    variables=ctx.variables,
                    get_global_variable=ctx.get_global_variable,
                ):
                    ctx.check_stop()
                    self.execute_children(ctx)
            else:
                while self.while_cond(ctx):
                    ctx.check_stop()
                    self.execute_children(ctx)

        elif self.do_while is not None:
            while True:
                ctx.check_stop()
                self.execute_children(ctx)
                if isinstance(self.do_while, str):
                    if not safe_eval_expr(
                        self.do_while,
                        variables=ctx.variables,
                        get_global_variable=ctx.get_global_variable,
                    ):
                        break
                else:
                    if not self.do_while(ctx):
                        break

        self._post_execute(ctx)

        ctx.emit_done(self.step_id)



class ConditionStep(Step):
    """조건 Step"""

    _class_name = "ConditionStep"

    def __init__(
        self,
        *,
        step_id: str,
        group_id: str,
        condition_type: str,
        condition: str | Callable | bool | None = True,
        args: dict[str, Any] | None = None,
        children: list | None = None,
    ):
        super().__init__(
            step_id=step_id,
            name=condition_type,
            func=None,
            args=args,
            condition_type=condition_type,
            condition=condition,
            children=children,
            group_id=group_id,
        )
        self.condition = condition
        self.condition_type = condition_type
        self.group_id = group_id
        self.args = args or {}
        self.children = children or []

    @staticmethod
    def from_dict(d) -> "ConditionStep":
        condition_type = d.get("conditionType") or d.get("args", {}).get("condition_type")
        condition = d.get("condition") or d.get("args", {}).get("condition", True)

        if condition == "":
            condition = True

        if condition_type is None:
            raise RuntimeError("condition_type is required")

        return ConditionStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            group_id=d.get("groupId"),
            condition_type=condition_type,
            condition=condition,
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.condition_type,
            "conditionType": self.condition_type,
            "groupId": self.group_id,
            "condition": self.condition,
            "children": [child.to_dict() for child in self.children],
        }

    def execute(self, ctx: ExecutionContext):
        """조건이 참일 때만 자식 Step 실행"""
        condition_map = ctx.data.get("condition_map", {})
        key = f"{self.group_id}.{ctx.current_depth}"

        condition_result = condition_map.get(key, None)

        if self.condition_type == "If":
            condition_map[key] = False
            condition_result = False
        else:
            if self.condition_type == "ElseIf" and condition_result is None:
                raise RuntimeError("ElseIf must be preceded by an If or ElseIf")

            if self.condition_type == "Else" and self.condition is not None:
                raise RuntimeError("Else must not have a condition")

        if condition_result:
            return

        self._pre_execute(ctx, reset_condition_result=False)

        ctx.emit_next(self.step_id)

        ctx.check_stop()

        if isinstance(self.condition, bool):
            if self.condition:
                ctx.data["condition_result"] = True

                self._post_execute(ctx)

                ctx.emit_done(self.step_id)

                self.execute_children(ctx)
                return

        elif isinstance(self.condition, str):
            if safe_eval_expr(
                self.condition,
                variables=ctx.variables,
                get_global_variable=ctx.get_global_variable,
            ):
                ctx.data["condition_result"] = True

                self._post_execute(ctx)

                ctx.emit_done(self.step_id)

                self.execute_children(ctx)
                return

        elif self.condition is not None and not callable(self.condition) and self.condition():
            ctx.data["condition_result"] = True

            self._post_execute(ctx)

            ctx.emit_done(self.step_id)

            self.execute_children(ctx)
            return


