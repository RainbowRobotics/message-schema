import importlib
import sys
import time
import uuid
from collections.abc import Callable
from multiprocessing import Event
from pathlib import Path
from typing import Any, Literal

from rb_schemas.sdk import FlowManagerArgs
from rb_utils.flow_manager import call_with_matching_args, eval_value, safe_eval_expr

from rb_flow_manager.schema import RB_Flow_Manager_ProgramState

from .context import ExecutionContext
from .exception import (
    BreakFolder,
    BreakRepeat,
    ChangeSubTaskException,
    ContinueRepeat,
    FlowControlException,
    JumpToStepException,
    StopExecution,
    SubTaskHaltException,
)
from .utils import _resolve_arg_scope_value


class Step:
    """실행 단계 (children을 가질 수 있음)"""

    def __init__(
        self,
        *,
        step_id: str,
        name: str,
        variable: dict[str, Any] | None = None,
        task_id: str | None = None,
        disabled: bool | None = None,
        method: str | None = None,
        func: Callable | None = None,
        post_func: Callable | None = None,
        done_script: Callable | str | None = None,
        children: list["Step"] | None = None,
        func_name: str | None = None,
        post_func_name: str | None = None,
        not_ast_eval: bool | None = None,
        args: dict[str, Any] | None = None,
        post_args: dict[str, Any] | None = None,
        **kwargs,
    ):
        self._class_name = getattr(self, "_class_name", "Step")
        self._start_ts: float = 0
        self.step_id = step_id
        self.name = name
        self.task_id = task_id
        self.variable = variable
        self.disabled = disabled
        self.method = method
        self.func = func
        self.post_func = post_func
        self.func_name = func_name
        self.post_func_name = post_func_name
        self.not_ast_eval = not_ast_eval
        self.done_script = done_script
        self.children = children or []
        self.args = args or {}
        self.post_args = post_args or {}
        self.kwargs = kwargs
        self.func = func

    @staticmethod
    def from_dict(d) -> "Step":
        if d.get("method") == "Folder":
            return FolderStep.from_dict(d)

        if d.get("method") == "Repeat":
            return RepeatStep.from_dict(d)

        if d.get("method") in ["If", "ElseIf", "Else", "Condition", "Case"]:
            return ConditionStep.from_dict(d)

        if d.get("method") == "Jump":
            return JumpToStep.from_dict(d)

        if d.get("method") == "Break":
            return BreakStep.from_dict(d)

        if d.get("method") == "SubTask":
            return SubTaskStep.from_dict(d)

        if d.get("funcName") == "rb_base_sdk.call_event_tree":
            return CallEventStep.from_dict(d)

        if d.get("method") == "CallEvent":
            return CallEventStep.from_dict(d)

        if d.get("method") == "Sync":
            return SyncStep.from_dict(d)

        return Step(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            memo=d.get("memo"),
            variable=d.get("variable"),
            task_id=d.get("taskId"),
            func_name=d.get("funcName"),
            post_func_name=d.get("postFuncName"),
            not_ast_eval=d.get("notAstEval"),
            disabled=d.get("disabled"),
            args=d.get("args") or {},
            post_args=d.get("postArgs") or {},
            done_script=d.get("doneScript"),
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "taskId": self.task_id,
            "disabled": self.disabled,
            "variable": self.variable,
            "funcName": self.func_name,
            "postFuncName": self.post_func_name,
            "args": self.args,
            "postArgs": self.post_args,
            "method": "Step",
            "children": [child.to_dict() for child in self.children],
        }

    def to_py_string(self, depth: int = 0):
        raw_expr_strings = {"event_sub_tree_list"}

        def _indent(s: str, n: int) -> str:
            pad = " " * n
            return "\n".join(pad + line if line else line for line in s.splitlines())

        def format_value(v: Any, level: int = 0) -> str:
            if isinstance(v, str):
                if v in raw_expr_strings:
                    return v
                return repr(v)
            if isinstance(v, dict):
                return format_dict(v, level)
            if isinstance(v, list):
                return "[" + ", ".join(format_value(x, level) for x in v) + "]"
            if callable(v):
                return getattr(v, "__name__", str(v))
            return repr(v)

        def format_dict(d: dict, level: int = 0) -> str:
            pad = " " * (4 * (level + 1))
            closing_pad = " " * (4 * level)
            items = []
            for k, v in d.items():
                items.append(f"{pad}{repr(k)}: {format_value(v, level + 1)},")
            return "{\n" + "\n".join(items) + "\n" + f"{closing_pad}" + "}"

        if self.children:
            inner_steps = [child.to_py_string(depth + 8) for child in self.children]
            inner_src = "\n".join(inner_steps)

            children_block = "[\n" + inner_src + "\n" + (" " * (depth + 4)) + "]"
        else:
            children_block = "[]"

        variable_block = ""
        if self.variable:
            v = format_dict(self.variable)
            variable_block = _indent(f"variable={v},\n\n", depth + 4)

        # args block
        args_block = ""
        if self.args:
            a = format_dict(self.args)
            args_block = _indent(f"args={a},\n\n", depth + 4)

        post_args_block = ""
        if self.post_args:
            a = format_dict(self.post_args)
            post_args_block = _indent(f"post_args={a},\n\n", depth + 4)

        other_kwargs_block = ""
        if self.kwargs:
            for k, v in self.kwargs.items():
                other_kwargs_block += (
                    _indent(f"{k}={repr(v)},\n\n", depth + 4) if v is not None else ""
                )

        children_src = ""
        if self.children:
            children_src = (" " * (depth + 4)) + f"children={children_block},\n"

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
            + (
                _indent(f"func={format_value(self.func)},\n\n", depth + 4)
                if self.func is not None and self.func_name is None
                else ""
            )
            + args_block
            + (
                _indent(f"post_func_name={repr(self.post_func_name)},\n\n", depth + 4)
                if self.post_func_name
                else ""
            )
            + post_args_block
            + (
                _indent(f"done_script={repr(self.done_script)},\n", depth + 4)
                if self.done_script is not None
                else ""
            )
            + other_kwargs_block
            + children_src
            + (" " * depth)
            + f"){',' if depth > 0 else ''}"
        )

    def _pre_execute(
        self, ctx: ExecutionContext, *, reset_condition_result: bool = True, is_skip: bool = False
    ):
        # parent/self owner 변수 스냅샷을 매 스텝 직전에 동기화
        if hasattr(ctx, "sync_state_variables"):
            ctx.sync_state_variables()

        if reset_condition_result:
            condition_map: dict[str, bool] = ctx.data.get("condition_map", {})

            for key in list(condition_map.keys()):
                condition_depth = int(key.split(".")[-1])

                if condition_depth > ctx.current_depth:
                    condition_map.pop(key)
                ctx.data["condition_map"] = condition_map

        ctx.state_dict["current_step_id"] = self.step_id
        ctx.state_dict["ignore_stop"] = False
        ctx.push_args(self.args)

        if not is_skip:
            ctx.step_barrier()

            self._start_ts = time.monotonic()

    def _post_execute(self, ctx: ExecutionContext, *, ignore_step_interval: bool = False):
        if not ignore_step_interval:
            min_step_interval = getattr(ctx, "min_step_interval", 0)
            elapsed = time.monotonic() - self._start_ts
            remaining = min_step_interval - elapsed

            if remaining > 0 and ctx.step_num != 1:
                time.sleep(remaining)

        ctx.step_num += 1

    def execute_children(self, ctx: ExecutionContext, *, target_step_id: str | None = None):
        """자식 Step들을 순차적으로 실행"""
        has_children = len(self.children) > 0
        if has_children:
            ctx.current_depth += 1

        current_target_step_id = target_step_id

        try:
            for child in self.children:
                before_scope_len = len(getattr(ctx, "_arg_scope", []))
                try:
                    child_target_step_id = getattr(child, "target_step_id", None)
                    if (
                        child_target_step_id is not None
                        and child_target_step_id == current_target_step_id
                    ):
                        continue

                    if current_target_step_id is not None and child.step_id == current_target_step_id:
                        current_target_step_id = None
                        ctx.data["finding_jump_to_step"] = False

                    child.execute(ctx, target_step_id=current_target_step_id)
                finally:
                    after_scope_len = len(getattr(ctx, "_arg_scope", []))
                    while after_scope_len > before_scope_len:
                        ctx.pop_args()
                        after_scope_len -= 1
        finally:
            if has_children:
                ctx.current_depth -= 1

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None, _post_run: bool = False):
        """단계 실행"""
        is_skip = target_step_id is not None and target_step_id != self.step_id


        self._pre_execute(ctx, is_skip=is_skip)

        if not is_skip:
            ctx.emit_next(self.step_id)

            ctx.check_stop()

        done_called = False

        fn = self.func if not _post_run else self.post_func
        func_name = _resolve_arg_scope_value(self.func_name, ctx) if self.func_name else None
        args = _resolve_arg_scope_value(self.args, ctx) if self.args else {}

        if _post_run:
            func_name = _resolve_arg_scope_value(self.post_func_name, ctx) if self.post_func_name else None
            args = _resolve_arg_scope_value(self.post_args, ctx) if self.post_args else self.args or {}


        if fn is None and func_name and not self.disabled and not is_skip:
            fn = ctx.get_sdk_function(func_name)
            if fn is None:
                raise StopExecution(f"unknown flow function: {func_name}")

        if fn is not None and not self.disabled and not is_skip:
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

                ctx.update_vars_to_state_dict()

                done_event.set()

            flow_manager_args = FlowManagerArgs(ctx=ctx, args=self.args or {}, done=done)

            eval_args = {}

            try:
                for k, v in args.items():
                    if self.not_ast_eval:
                        eval_args[k] = v
                    else:
                        eval_args[k] = eval_value(
                            v, variables=ctx.variables, get_global_variable=ctx.get_global_variable
                        )

                if self.variable:
                    resolved_variables = dict(self.variable)

                    for k, v in self.variable.items():
                        if self.not_ast_eval:
                            resolved_variables[k] = v
                        else:
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

                        resolved_variables[k] = v

                    # 원본 step 정의(self.variable)를 mutate하지 않고 실행 시점 값만 반영한다.
                    ctx.update_local_variables(resolved_variables)

                if "robot_model" not in eval_args:
                    eval_args["robot_model"] = ctx.state_dict.get("robot_model", None)

                eval_args["target_step_id"] = target_step_id
                eval_args["_post_run"] = _post_run

            except FlowControlException as e:
                raise e
            except RuntimeError as e:
                if "Execution stopped by user" not in str(e):
                    ctx.emit_error(self.step_id, RuntimeError(str(e)))
                print(f"[{ctx.process_id}] Step '{self.name}' error: {e}", flush=True)
                raise e
            except Exception as e:  # noqa: BLE001
                if "Execution stopped by user" not in str(e):
                    ctx.emit_error(self.step_id, e)

                print(f"[{ctx.process_id}] Step '{self.name}' error: {e}", flush=True)
                raise e

            try:
                call_with_matching_args(fn, **eval_args, flow_manager_args=flow_manager_args)
            except FlowControlException as e:
                raise e
            except RuntimeError as e:
                if "Execution stopped by user" not in str(e):
                    ctx.emit_error(self.step_id, RuntimeError(str(e)))
                print(f"[{ctx.process_id}] Step '{self.name}' error: {e}", flush=True)
                raise RuntimeError(str(e)) from e
            except Exception as e:  # noqa: BLE001
                if "Execution stopped by user" not in str(e):
                    ctx.emit_error(self.step_id, e)
                print(f"[{ctx.process_id}] Step '{self.name}' error: {e}", flush=True)
                raise e

            while not done_event.wait(timeout=0.01):
                ctx.check_stop()

        self._post_execute(ctx, ignore_step_interval=self.disabled or is_skip)

        if not is_skip:
            ctx.emit_done(self.step_id)

        if not _post_run:
            self.execute_children(ctx, target_step_id=target_step_id)

        if not _post_run and (self.post_func is not None or self.post_func_name is not None):
            self.execute(ctx, target_step_id=target_step_id, _post_run=True)


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
        disabled: bool | None = None,
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
            disabled=d.get("disabled"),
            children=[Step.from_dict(child) for child in (d.get("children") or [])],
            args=d.get("args") or {},
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "funcName": self.func_name,
            "func": self.func,
            "disabled": self.disabled,
            "method": self.method,
            "children": [child.to_dict() for child in self.children],
            "args": self.args,
        }

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None):
        if self.disabled:
            return

        ctx.enter_folder()

        try:
            super().execute(ctx, target_step_id=target_step_id)
        except BreakFolder:
            return
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
        count: int | None = None,
        while_cond: str | Callable | None = None,
        do_while: str | Callable | None = None,
        args: dict[str, Any] | None = None,
        children: list | None = None,
        disabled: bool | None = None,
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
            disabled=disabled,
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
        disabled = d.get("disabled")

        if count is None and while_cond is None and do_while is None:
            raise RuntimeError("count, while_cond, or do_while is required")

        return RepeatStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            count=count,
            while_cond=while_cond,
            do_while=do_while,
            disabled=disabled,
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

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None):
        """지정된 횟수만큼 자식 Step 반복 실행"""
        if self.disabled:
            return

        is_skip = target_step_id is not None and target_step_id != self.step_id

        def find_target_step(step: Step) -> bool:
            if step.step_id == target_step_id:
                return True
            return any(find_target_step(child) for child in step.children)

        if target_step_id is not None:
            is_skip = is_skip or find_target_step(self)

        if is_skip:
            self.execute_children(ctx, target_step_id=target_step_id)
            self._post_execute(ctx, ignore_step_interval=True)
            return

        if self.count is not None:
            if int(self.count) == -1:
                i = 0
                while True:
                    i += 1

                    self._pre_execute(ctx, is_skip=is_skip)
                    ctx.emit_next(self.step_id)
                    ctx.check_stop()

                    ctx.data["repeat_index"] = i

                    try:
                        self.execute_children(ctx, target_step_id=target_step_id)
                        self._post_execute(ctx, ignore_step_interval=True)
                        ctx.emit_done(self.step_id)
                    except BreakRepeat:
                        break
                    except ContinueRepeat:
                        continue
            else:
                for i in range(int(self.count)):
                    self._pre_execute(ctx, is_skip=is_skip)
                    ctx.emit_next(self.step_id)
                    ctx.check_stop()

                    ctx.data["repeat_index"] = i

                    try:
                        self.execute_children(ctx, target_step_id=target_step_id)
                        self._post_execute(ctx, ignore_step_interval=True)
                        ctx.emit_done(self.step_id)
                    except BreakRepeat:
                        break
                    except ContinueRepeat:
                        continue

        elif self.while_cond is not None:
            if isinstance(self.while_cond, str):
                while safe_eval_expr(
                    self.while_cond,
                    variables=ctx.variables,
                    get_global_variable=ctx.get_global_variable,
                ):
                    try:
                        self._pre_execute(ctx, is_skip=is_skip)
                        ctx.emit_next(self.step_id)
                        ctx.check_stop()
                        self.execute_children(ctx, target_step_id=target_step_id)
                        self._post_execute(ctx, ignore_step_interval=True)
                        ctx.emit_done(self.step_id)
                    except BreakRepeat:
                        break
                    except ContinueRepeat:
                        continue
            else:
                while self.while_cond(ctx):
                    try:
                        self._pre_execute(ctx, is_skip=is_skip)
                        ctx.emit_next(self.step_id)
                        ctx.check_stop()
                        self.execute_children(ctx, target_step_id=target_step_id)
                        self._post_execute(ctx, ignore_step_interval=True)
                        ctx.emit_done(self.step_id)
                    except BreakRepeat:
                        break
                    except ContinueRepeat:
                        continue

        elif self.do_while is not None:
            while True:
                try:
                    self._pre_execute(ctx, is_skip=is_skip)
                    ctx.emit_next(self.step_id)
                    ctx.check_stop()
                    self.execute_children(ctx, target_step_id=target_step_id)
                    self._post_execute(ctx, ignore_step_interval=True)
                    ctx.emit_done(self.step_id)
                except BreakRepeat:
                    break
                except ContinueRepeat:
                    continue
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


class ConditionStep(Step):
    """조건 Step"""

    _class_name = "ConditionStep"

    def __init__(
        self,
        *,
        step_id: str,
        group_id: str,
        name: str,
        condition_type: str,
        condition: str | Callable | bool | None = True,
        disabled: bool | None = None,
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
            disabled=disabled,
        )
        self.condition = condition
        self.condition_type = condition_type
        self.group_id = group_id
        self.args = args or {}
        self.children = children or []
        self.disabled = disabled

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
            name=d.get("conditionType") or d.get("name"),
            condition_type=condition_type,
            condition=condition,
            disabled=d.get("disabled"),
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

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None):
        """조건이 참일 때만 자식 Step 실행"""
        if self.disabled:
            return

        is_skip = target_step_id is not None and target_step_id != self.step_id

        def find_target_step(step: Step) -> bool:
            if step.step_id == target_step_id:
                return True
            return any(find_target_step(child) for child in step.children)

        if target_step_id is not None:
            is_skip = is_skip or find_target_step(self)

        # 점프 탐색 중이면 조건 평가를 건너뛰고 target child 탐색만 수행
        if is_skip:
            self.execute_children(ctx, target_step_id=target_step_id)
            self._post_execute(ctx, ignore_step_interval=True)
            return

        condition_map = ctx.data.get("condition_map")

        if condition_map is None:
            condition_map = {}
            ctx.data["condition_map"] = condition_map

        key = f"{self.group_id}.{ctx.current_depth}"

        condition_result = condition_map.get(key)

        if (
            self.condition_type == "Case"
            and condition_result is None
            or self.condition_type == "If"
        ):
            condition_map[key] = False
            condition_result = False
        else:
            if self.condition_type == "ElseIf" and condition_result is None:
                raise RuntimeError("ElseIf must be preceded by an If or ElseIf")

            if self.condition_type == "Else" and condition_result is None:
                raise RuntimeError("Else must not have a condition")

        if condition_result:
            return

        self._pre_execute(ctx, reset_condition_result=False)

        ctx.emit_next(self.step_id)

        ctx.check_stop()

        def post_execute_and_execute_children():
            ctx.data["condition_map"][key] = True

            if not is_skip:
                self._post_execute(ctx)
                ctx.emit_done(self.step_id)

            self.execute_children(ctx, target_step_id=target_step_id)

        if self.condition_type == "Else" and not condition_result:
            post_execute_and_execute_children()
            return

        if isinstance(self.condition, bool):
            if self.condition:
                post_execute_and_execute_children()
                return

        elif isinstance(self.condition, str):
            condition_expr = self.condition

            if self.condition_type == "Case":
                parent_switch_value = ctx.lookup("switch_value")
                condition_expr = condition_expr.replace("$parent.switch_value", parent_switch_value)

            if safe_eval_expr(
                condition_expr,
                variables=ctx.variables,
                get_global_variable=ctx.get_global_variable,
            ):
                post_execute_and_execute_children()
                return

        elif self.condition is not None and callable(self.condition) and self.condition():
            post_execute_and_execute_children()
            return


class BreakStep(Step):
    """가장 가까운 RepeatStep 실행을 중단"""

    _class_name = "BreakStep"

    def __init__(self, *, step_id: str, name: str, break_type: str, disabled: bool | None = None):
        super().__init__(step_id=step_id, name=name, break_type=break_type, disabled=disabled)
        self.break_type = break_type

    @staticmethod
    def from_dict(d) -> "BreakStep":
        return BreakStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            break_type=d.get("breakType") or d.get("args", {}).get("break_type"),
            disabled=d.get("disabled"),
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "breakType": self.break_type,
            "disabled": self.disabled,
        }

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None):
        if self.disabled:
            return

        is_skip = target_step_id is not None and target_step_id != self.step_id

        self._pre_execute(ctx, is_skip=is_skip)

        if not is_skip:
            ctx.emit_next(self.step_id)

            ctx.check_stop()

        self._post_execute(ctx, ignore_step_interval=True)

        if not is_skip:
            ctx.emit_done(self.step_id)

            if self.break_type == "BREAK":
                raise BreakRepeat()
            elif self.break_type == "CONTINUE":
                raise ContinueRepeat()


class JumpToStep(Step):
    """특정 step_id로 점프"""

    _class_name = "JumpToStep"

    def __init__(
        self,
        *,
        step_id: str,
        name: str,
        target_step_id: str | None = None,
        disabled: bool | None = None,
    ):
        super().__init__(
            step_id=step_id, name=name, target_step_id=target_step_id, disabled=disabled
        )
        self.target_step_id = target_step_id

    @staticmethod
    def from_dict(d) -> "JumpToStep":
        return JumpToStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            target_step_id=d.get("targetStepId"),
            disabled=d.get("disabled"),
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "targetStepId": self.target_step_id,
            "disabled": self.disabled,
        }

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None):
        if self.disabled:
            return

        self._pre_execute(ctx)

        ctx.emit_next(self.step_id)

        ctx.check_stop()

        self._post_execute(ctx, ignore_step_interval=True)

        ctx.emit_done(self.step_id)

        if self.target_step_id is not None:
            raise JumpToStepException(target_step_id=self.target_step_id)


class SubTaskStep(Step):
    """서브 태스크 Step"""

    _class_name = "SubTaskStep"

    def __init__(
        self,
        *,
        step_id: str,
        name: str,
        sub_task_type: Literal["INSERT", "CHANGE"],
        sub_task_script_path: str,
        disabled: bool | None = None,
        children: list | None = None,
    ):
        super().__init__(
            step_id=step_id,
            name=name,
            sub_task_type=sub_task_type,
            sub_task_script_path=sub_task_script_path,
            disabled=disabled,
        )
        self.sub_task_type = sub_task_type
        self.sub_task_script_path = sub_task_script_path

    @staticmethod
    def from_dict(d) -> "SubTaskStep":
        return SubTaskStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d["name"],
            sub_task_type=d.get("subTaskType"),
            sub_task_script_path=d.get("subTaskScriptPath"),
            disabled=d.get("disabled"),
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "subTaskType": self.sub_task_type,
            "subTaskScriptPath": self.sub_task_script_path,
            "disabled": self.disabled,
        }

    def _load_module_from_file(self, py_path: str):
        path = Path(py_path)

        if not path.exists():
            raise RuntimeError(f"script not found: {path}")

        module_name = f"subtask_{path.stem}"

        spec = importlib.util.spec_from_file_location(module_name, str(path))
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot create module spec: {path}")

        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)

        return module

    def _load_sub_task_trees(self):
        module = self._load_module_from_file(self.sub_task_script_path)
        tree = getattr(module, "tree", None)
        post_tree = getattr(module, "post_tree", None)

        if module is None:
            raise RuntimeError(f"'{self.sub_task_script_path}' not found")

        if tree is None:
            raise RuntimeError(f"'{self.sub_task_script_path}' must define global `tree`")

        return tree, post_tree

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None):
        is_skip = target_step_id is not None and target_step_id != self.step_id

        if self.disabled or is_skip:
            self._post_execute(ctx, ignore_step_interval=True)
            return

        self._pre_execute(ctx, is_skip=is_skip)

        if self.disabled:
            return

        tree, post_tree = self._load_sub_task_trees()

        sub_task_tree: Step = tree
        sub_task_post_tree: Step | None = post_tree

        ctx.emit_next(self.step_id)

        ctx.check_stop()

        if self.sub_task_type == "INSERT":
            try:
                # 서브태스크 트리 정보를 state_dict에 저장
                ctx.emit_subtask_sync_register(sub_task_tree, sub_task_post_tree, "INSERT")
                time.sleep(0.05)

                sub_task_list = ctx.state_dict.get("sub_task_list", [])
                sub_task_list.append(
                    {"task_id": sub_task_tree.step_id, "sub_task_type": self.sub_task_type}
                )
                ctx.state_dict["sub_task_list"] = sub_task_list

                ctx.emit_sub_task_start(sub_task_tree.step_id, self.sub_task_type)

                sub_task_tree.execute_children(ctx, target_step_id=target_step_id)
            except SubTaskHaltException:
                pass
            finally:
                ctx.emit_subtask_sync_unregister(sub_task_tree, sub_task_post_tree, "INSERT")
                time.sleep(0.05)

                sub_task_list = ctx.state_dict.get("sub_task_list", [])
                if len(sub_task_list) > 0:
                    sub_task_list.pop()
                    ctx.state_dict["sub_task_list"] = sub_task_list
                    time.sleep(0.1)

                ctx.emit_sub_task_done(sub_task_tree.step_id, self.sub_task_type)

        elif self.sub_task_type == "CHANGE":
            ctx.step_num = 0
            raise ChangeSubTaskException(
                task_id=sub_task_tree.step_id,
                sub_task_tree=sub_task_tree,
                sub_task_post_tree=sub_task_post_tree,
            )

        self._post_execute(ctx, ignore_step_interval=True)

        ctx.emit_done(self.step_id)


class CallEventStep(Step):
    """이벤트 서브 태스크 Step (별도 프로세스 실행)"""

    _class_name = "CallEventStep"

    def __init__(
        self,
        *,
        step_id: str,
        name: str,
        process_id: str | None = None,
        index: int | None = None,
        run_mode: Literal["SYNC", "ASYNC"] | None = None,
        disabled: bool | None = None,
        args: dict[str, Any] | None = None,
    ):
        super().__init__(
            step_id=step_id,
            name=name,
            method="CallEvent",
            args=args,
            process_id=process_id,
            disabled=disabled,
        )
        self.disabled = disabled
        self.args = args or {}

        self.process_id = process_id if process_id is not None else self.args.get("process_id")
        self.index = index if index is not None else int(self.args.get("index", 0))
        self.run_mode = run_mode if run_mode is not None else self.args.get("run_mode", "ASYNC")

    @staticmethod
    def from_dict(d) -> "CallEventStep":
        args = d.get("args", {}) or {}

        process_id = d.get("processId") or args.get("process_id")
        index = args.get("index", 0)
        run_mode = (d.get("runMode") or args.get("run_mode") or "ASYNC")

        return CallEventStep(
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            name=d.get("name") or "CallEvent",
            process_id=process_id,
            index=int(index),
            run_mode=str(run_mode).upper(),
            disabled=d.get("disabled"),
            args=args,
        )

    def to_dict(self):
        return {
            "stepId": self.step_id,
            "name": self.name,
            "method": "CallEvent",
            "processId": self.process_id,
            "index": self.index,
            "runMode": self.run_mode,
            "disabled": self.disabled,
            "args": self.args,
        }

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None, _post_run: bool = False):
        is_skip = target_step_id is not None and target_step_id != self.step_id

        self._pre_execute(ctx, is_skip=is_skip)

        if self.disabled or is_skip:
            self._post_execute(ctx, ignore_step_interval=True)
            return

        ctx.emit_next(self.step_id)
        ctx.check_stop()

        trees = ctx.state_dict.get("event_sub_tree_list", [])

        # event_sub_tree_list가 없거나 비어있으면 스킵
        if not trees:
            print("[CallEventStep] No event_sub_tree_list, skipping", flush=True)
            self._post_execute(ctx, ignore_step_interval=True)
            ctx.emit_done(self.step_id)
            return

        tree: Step | None = None
        if isinstance(trees, list) and 0 <= self.index < len(trees):
            tree = trees[self.index]
            if isinstance(tree, dict):
                tree = Step.from_dict(tree)

        if tree is None:
            print("[CallEventStep] Cannot resolve tree, skipping", flush=True)
            self._post_execute(ctx, ignore_step_interval=True)
            ctx.emit_done(self.step_id)
            return

        task_id = (
            self.process_id
            if self.process_id is not None
            else (tree.task_id if tree.task_id is not None else tree.step_id)
        )

        if task_id is None:
            print("[CallEventStep] No valid task_id, skipping", flush=True)
            self._post_execute(ctx, ignore_step_interval=True)
            ctx.emit_done(self.step_id)
            return

        print(f"[CallEventStep] Spawning event task: {task_id}", flush=True)

        executor_managed = bool(ctx.state_dict.get("executor_managed", False))
        call_seq: int | None = None

        if executor_managed:
            call_seq = int(ctx.state_dict.get("_event_call_seq", 0)) + 1
            ctx.state_dict["_event_call_seq"] = call_seq
            pending_map = dict(ctx.state_dict.get("event_start_pending_map", {}))
            pending_map[task_id] = call_seq
            ctx.state_dict["event_start_pending_map"] = pending_map

        ctx.emit_event_sub_task_start(
            event_task_id=task_id,
            event_tree=tree,
            step_id=self.step_id,
            run_mode=self.run_mode,
            call_seq=call_seq,
        )

        if executor_managed:
            # ASYNC/SYNC 모두 이벤트 프로세스 시작(ready) ack까지는 대기
            while dict(ctx.state_dict.get("event_start_pending_map", {})).get(task_id) == call_seq:
                ctx.check_stop()
                time.sleep(0.01)

            rejected_map = dict(ctx.state_dict.get("event_start_rejected_map", {}))
            rejected = rejected_map.get(task_id)
            if isinstance(rejected, dict) and rejected.get("call_seq") == call_seq:
                message = str(rejected.get("message") or f"Event sub task already running: {task_id}")
                rejected_map.pop(task_id, None)
                ctx.state_dict["event_start_rejected_map"] = rejected_map
                ctx.emit_error(self.step_id, RuntimeError(message))
                raise RuntimeError(message)

        if executor_managed and self.run_mode == "SYNC":
            # RUNNING 상태로 이벤트 자식 완료 대기
            sync_children = list(ctx.state_dict.get("event_sync_children", []))
            if task_id not in sync_children:
                sync_children.append(task_id)
                ctx.state_dict["event_sync_children"] = sync_children

            while task_id in list(ctx.state_dict.get("event_sync_children", [])):
                ctx.check_stop()
                time.sleep(0.01)

        self._post_execute(ctx, ignore_step_interval=True)
        ctx.emit_done(self.step_id)

class SyncStep(Step):
    """
    여러 프로세스 간 동기화를 위한 Step

    프로세스가 동적으로 추가/제거되어도 올바르게 동작하는 barrier 패턴 구현
    - 지정된 flag에서 모든 참가 프로세스가 도착할 때까지 대기
    - phase 기반으로 같은 flag를 반복 사용 가능 (loop 내에서도 작동)
    - 프로세스 추가 시 parties_next 증가
    - 프로세스 제거 시 parties_next 감소 및 대기 중인 프로세스 깨우기
    """

    _class_name = "SyncStep"

    def __init__(
        self,
        flag: str | None = None,
        timeout: float | None = None,
        name: str | None = None,
        step_id: str | None = None,
        args: dict[str, Any] | None = None,
        disabled: bool | None = None,
    ):
        """
        Args:
            flag: 동기화 플래그 (같은 플래그를 사용하는 프로세스들이 동기화됨)
            timeout: 최대 대기 시간 (초). None이면 무제한 대기
            name: Step 이름 (기본값: f"Sync({flag})")
            step_id: Step ID
            disabled: Step 비활성화 여부
        """
        super().__init__(
            step_id=step_id,
            name=name,
            args=args,
            disabled=disabled,
        )
        _args = args or {}

        self.disabled = disabled
        self.args = _args
        self.flag = flag if flag is not None else _args.get("flag")
        self.timeout = timeout if timeout is not None else _args.get("timeout")

        if not self.flag:
            raise ValueError("SyncStep requires 'flag' parameter")

    def execute(self, ctx: ExecutionContext, *, target_step_id: str | None = None, _post_run: bool = False):
        """
        동기화 barrier 실행

        동작 원리:
        1. 프로세스가 도착하면 arrived 카운트 증가
        2. arrived >= parties_cur이면 마지막 도착자
           - phase를 증가시켜 다음 라운드로 진입
           - parties_cur를 parties_next로 업데이트 (동적 변경 반영)
           - 모든 대기 프로세스를 notify_all()로 깨움
        3. 마지막 도착자가 아니면 대기
           - phase가 증가할 때까지 대기
           - 대기 중 프로세스 제거로 인해 arrived >= parties가 된 경우도 처리

        Raises:
            RuntimeError: Sync 인프라가 초기화되지 않았거나 flag가 등록되지 않은 경우
            TimeoutError: timeout 시간 내에 동기화가 완료되지 않은 경우
        """
        if self.disabled:
            return

        is_skip = target_step_id is not None and target_step_id != self.step_id

        self._pre_execute(ctx, is_skip=is_skip)

        if is_skip:
            self._post_execute(ctx, ignore_step_interval=True)
            return

        ctx.emit_next(self.step_id)
        ctx.check_stop()

        if ctx.sync_lock is None or ctx.sync_cond is None:
            raise RuntimeError("Sync infrastructure not initialized")

        pid = ctx.process_id
        flag = self.flag

        deadline = None
        if self.timeout is not None:
            deadline = time.time() + float(self.timeout)

        acquired = ctx.sync_cond.acquire(timeout=1.0)
        if not acquired:
            raise RuntimeError(f"SyncStep({flag}): sync lock stuck")
        try:
            state = dict(ctx.sync_state.get(flag, {}))
            if not state:
                raise RuntimeError(
                    f"SyncStep({flag}): Sync state not initialized. "
                    f"Process {pid} may not be registered for this flag."
                )

            current_phase = int(state.get("phase", 0))

            # 현재 참가자 수는 상태 캐시가 아니라 members 실측값을 우선 사용한다.
            members = list(ctx.sync_members.get(flag, []))
            parties_cur = len(members)
            state["parties_cur"] = parties_cur
            state["parties_next"] = int(state.get("parties_next", parties_cur))
            ctx.sync_state[flag] = state

            if parties_cur <= 1:
                raise RuntimeError(
                    f"SyncStep(Flag = > {flag}): only one participant. "
                    "This sync must have at least 2 participants."
                )


            arrived = int(state.get("arrived", 0)) + 1
            state["arrived"] = arrived
            ctx.sync_state[flag] = state

            print(f"[Sync {flag}] {pid} arrived ({arrived}/{parties_cur}) phase={current_phase}", flush=True)

            # 마지막 도착자
            if arrived >= parties_cur:
                # 마지막 도착자
                next_parties = int(state.get("parties_next", parties_cur))
                state["phase"] = current_phase + 1
                state["arrived"] = 0
                state["parties_cur"] = next_parties
                # parties_next는 이미 레지스트리가 관리하는 값 => 덮지 않아도 됨
                ctx.sync_state[flag] = state
                ctx.sync_cond.notify_all()
            else:
                # 대기
                while True:
                    ctx.check_stop()

                    if ctx.state_dict.get("step_mode", False) and ctx.state_dict.get("state") == RB_Flow_Manager_ProgramState.RUNNING:
                        ctx.emit_pause(self.step_id, is_wait=True)

                    if deadline is not None:
                        remaining = deadline - time.time()
                        if remaining <= 0:
                            break
                        wait_timeout = min(remaining, 0.1)
                    else:
                        wait_timeout = 0.1

                    # wait()는 내부적으로 release -> wait -> reacquire를 수행
                    ctx.sync_cond.wait(timeout=wait_timeout)

                    updated = dict(ctx.sync_state.get(flag, {}))
                    updated_phase = int(updated.get("phase", 0))

                    # phase가 넘어갔으면 해제
                    if updated_phase > current_phase:
                        print(f"[Sync {flag}] {pid} released from phase {current_phase}", flush=True)
                        break

                    # 중간에 parties가 줄어서(프로세스 제거) 프로세스가 사실상 마지막이 된 경우 처리
                    updated_arrived = int(updated.get("arrived", 0))
                    updated_members = list(ctx.sync_members.get(flag, []))
                    updated_parties_cur = len(updated_members)
                    updated["parties_cur"] = updated_parties_cur
                    ctx.sync_state[flag] = updated

                    if updated_parties_cur > 0 and updated_arrived >= updated_parties_cur:
                        next_parties = int(updated.get("parties_next", updated_parties_cur))
                        if next_parties < 0:
                            next_parties = 0

                        updated["phase"] = updated_phase + 1
                        updated["arrived"] = 0
                        updated["parties_cur"] = next_parties
                        updated["parties_next"] = next_parties
                        ctx.sync_state[flag] = updated

                        print(
                            f"[Sync {flag}] {pid} becoming last due to parties reduction, "
                            f"phase {updated_phase} -> {updated_phase + 1}, "
                            f"parties: {updated_parties_cur} -> {next_parties}",
                            flush=True,
                        )
                        ctx.sync_cond.notify_all()
                        break

            print(f"[Sync {flag}] {pid} exiting sync", flush=True)

        finally:
            if acquired:
                ctx.sync_cond.release()



        self._post_execute(ctx, ignore_step_interval=True)
        ctx.emit_done(self.step_id)

    @staticmethod
    def from_dict(d) -> "SyncStep":
        args = d.get("args", {})

        flag = args.get("flag") or d.get("flag")
        timeout = args.get("timeout") or d.get("timeout")

        if not flag:
            raise ValueError(
                f"SyncStep requires 'flag' parameter. "
                f"Received dict: {d}, args: {args}"
            )

        return SyncStep(
            flag=flag,
            timeout=timeout,
            name=d.get("name"),
            step_id=str(d.get("stepId") or d.get("_id") or f"temp-{str(uuid.uuid4())}"),
            args=args,
            disabled=d.get("disabled", False),
        )

    def to_dict(self) -> dict:
        """Step을 딕셔너리로 직렬화"""
        base = super().to_dict()
        base.update({
            "method": "Sync",
            "flag": self.flag,
            "timeout": self.timeout,
            "args": {
                **(base.get("args") or {}),
                "flag": self.flag,
                "timeout": self.timeout,
            },
        })
        return base
