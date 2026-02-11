import importlib
import sys
import types
import unittest
from pathlib import Path
from typing import Any

PACKAGES_DIR = Path(__file__).resolve().parents[2]
SRC_PATHS = [
    PACKAGES_DIR / "rb_flow_manager" / "src",
    PACKAGES_DIR / "rb_utils" / "src",
    PACKAGES_DIR / "rb_schemas" / "src",
]
for src_path in SRC_PATHS:
    src = str(src_path)
    if src not in sys.path:
        sys.path.insert(0, src)


def _load_step_modules():
    prev_context_mod = sys.modules.get("rb_flow_manager.context")
    fake_context_mod = types.ModuleType("rb_flow_manager.context")

    class ExecutionContext:  # pragma: no cover - used only for import-time type reference
        pass

    fake_context_mod.ExecutionContext = ExecutionContext
    sys.modules["rb_flow_manager.context"] = fake_context_mod

    try:
        step_mod = importlib.import_module("rb_flow_manager.step")
        exc_mod = importlib.import_module("rb_flow_manager.exception")
        return step_mod, exc_mod
    finally:
        if prev_context_mod is None:
            sys.modules.pop("rb_flow_manager.context", None)
        else:
            sys.modules["rb_flow_manager.context"] = prev_context_mod


STEP_MOD, EXC_MOD = _load_step_modules()


class DummyExecutionContext:
    def __init__(self):
        self.process_id = "test-pid"
        self.state_dict: dict[str, Any] = {"robot_model": "test-model"}
        self.variables: dict[str, dict[str, Any]] = {"local": {}, "global": {}}
        self.sdk_functions = {}
        self.data: dict[str, Any] = {}
        self.current_depth = 0
        self.step_num = 1
        self.min_step_interval = 0.0
        self._folder_depth = 0
        self._arg_scope: list[dict[str, Any]] = []
        self.trace: list[str] = []
        self.events: list[tuple[str, str]] = []
        self.event_requests: list[dict[str, Any]] = []
        self.pause_calls: list[bool] = []
        self.subtask_events: list[tuple[str, str]] = []

    def update_vars_to_state_dict(self):
        return

    def update_local_variables(self, variables: dict[str, Any]):
        self.variables["local"].update(variables)
        self.update_vars_to_state_dict()

    def get_global_variable(self, var_name: str):
        return self.variables["global"].get(var_name)

    def step_barrier(self):
        return

    def check_stop(self):
        return

    def push_args(self, mapping: dict[str, Any] | None):
        self._arg_scope.append(mapping or {})

    def pop_args(self):
        if self._arg_scope:
            self._arg_scope.pop()

    def lookup(self, key: str) -> Any:
        idx = len(self._arg_scope) - 2
        while idx >= 0:
            scope = self._arg_scope[idx]
            if key in scope:
                return scope[key]
            idx -= 1
        raise KeyError(key)

    def emit_next(self, step_id: str):
        self.events.append(("next", step_id))

    def emit_done(self, step_id: str):
        self.events.append(("done", step_id))

    def emit_error(self, step_id: str, error: Exception):
        self.events.append(("error", f"{step_id}:{error}"))

    def emit_event_sub_task_start(
        self,
        *,
        event_task_id: str,
        event_tree,
        step_id: str | None = None,
        run_mode: str = "ASYNC",
        call_seq: int | None = None,
    ):
        self.event_requests.append(
            {
                "event_task_id": event_task_id,
                "event_tree": event_tree,
                "step_id": step_id,
                "run_mode": run_mode,
                "call_seq": call_seq,
            }
        )

    def pause(self, is_wait: bool = False):
        self.pause_calls.append(is_wait)

    def enter_folder(self):
        self._folder_depth += 1

    def leave_folder(self):
        self._folder_depth -= 1

    def break_folder(self):
        if self._folder_depth > 0:
            raise EXC_MOD.BreakFolder()

    def halt_sub_task(self):
        raise EXC_MOD.SubTaskHaltException()

    def emit_subtask_sync_register(self, sub_task_tree, post_tree=None, subtask_type: str = "INSERT"):
        return

    def emit_subtask_sync_unregister(self, sub_task_tree, post_tree=None, subtask_type: str = "INSERT"):
        return

    def emit_sub_task_start(self, task_id: str, sub_task_type: str):
        self.subtask_events.append(("start", task_id))

    def emit_sub_task_done(self, task_id: str, sub_task_type: str):
        self.subtask_events.append(("done", task_id))


def _run_root_with_jump(root_step, ctx):
    try:
        root_step.execute(ctx)
    except EXC_MOD.JumpToStepException as e:
        root_step.execute_children(ctx, target_step_id=e.target_step_id)


class StepControlFlowTest(unittest.TestCase):
    def test_composite_flow_with_repeat_condition_break_continue_folder_and_jump(self):
        Step = STEP_MOD.Step
        RepeatStep = STEP_MOD.RepeatStep
        ConditionStep = STEP_MOD.ConditionStep
        BreakStep = STEP_MOD.BreakStep
        FolderStep = STEP_MOD.FolderStep
        JumpToStep = STEP_MOD.JumpToStep

        ctx = DummyExecutionContext()

        def init_counter(*, flow_manager_args):
            flow_manager_args.ctx.update_local_variables({"i": 0})
            flow_manager_args.done()

        def inc_counter(*, flow_manager_args):
            i = flow_manager_args.ctx.variables["local"].get("i", 0) + 1
            flow_manager_args.ctx.update_local_variables({"i": i})
            flow_manager_args.ctx.trace.append(f"inc:{i}")
            flow_manager_args.done()

        def body(*, flow_manager_args):
            i = flow_manager_args.ctx.variables["local"]["i"]
            flow_manager_args.ctx.trace.append(f"body:{i}")
            flow_manager_args.done()

        def append_after_folder(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("after_folder")
            flow_manager_args.done()

        def append_skipped(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("skipped")
            flow_manager_args.done()

        def append_end(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("end")
            flow_manager_args.done()

        repeat_step = RepeatStep(
            step_id="rep",
            name="repeat",
            count=5,
            children=[
                Step(step_id="inc", name="inc", func=inc_counter),
                ConditionStep(
                    step_id="if_break",
                    group_id="g_break",
                    name="If",
                    condition_type="If",
                    condition="i == 2",
                    children=[BreakStep(step_id="brk", name="break", break_type="BREAK")],
                ),
                ConditionStep(
                    step_id="if_continue",
                    group_id="g_continue",
                    name="If",
                    condition_type="If",
                    condition="i == 1",
                    children=[BreakStep(step_id="cont", name="continue", break_type="CONTINUE")],
                ),
                Step(step_id="body", name="body", func=body),
            ],
        )

        folder = FolderStep(
            step_id="folder",
            name="folder",
            children=[
                repeat_step,
                Step(step_id="after_folder", name="after_folder", func=append_after_folder),
            ],
        )

        root = Step(
            step_id="root",
            name="root",
            children=[
                Step(step_id="init", name="init", func=init_counter),
                folder,
                JumpToStep(step_id="jump", name="jump", target_step_id="end"),
                Step(step_id="skipped", name="skipped", func=append_skipped),
                Step(step_id="end", name="end", func=append_end),
            ],
        )

        _run_root_with_jump(root, ctx)

        self.assertEqual(ctx.trace, ["inc:1", "inc:2", "after_folder", "end"])
        self.assertNotIn("body:1", ctx.trace)
        self.assertNotIn("skipped", ctx.trace)

    def test_nested_if_elseif_else_and_break_folder_in_repeat(self):
        Step = STEP_MOD.Step
        RepeatStep = STEP_MOD.RepeatStep
        ConditionStep = STEP_MOD.ConditionStep
        FolderStep = STEP_MOD.FolderStep

        ctx = DummyExecutionContext()

        def init_counter(*, flow_manager_args):
            flow_manager_args.ctx.update_local_variables({"i": 0})
            flow_manager_args.done()

        def inc_counter(*, flow_manager_args):
            i = flow_manager_args.ctx.variables["local"].get("i", 0) + 1
            flow_manager_args.ctx.update_local_variables({"i": i})
            flow_manager_args.done()

        def mark_if(*, flow_manager_args):
            i = flow_manager_args.ctx.variables["local"]["i"]
            flow_manager_args.ctx.trace.append(f"if:{i}")
            flow_manager_args.done()

        def mark_elseif(*, flow_manager_args):
            i = flow_manager_args.ctx.variables["local"]["i"]
            flow_manager_args.ctx.trace.append(f"elseif:{i}")
            flow_manager_args.done()

        def mark_else(*, flow_manager_args):
            i = flow_manager_args.ctx.variables["local"]["i"]
            flow_manager_args.ctx.trace.append(f"else:{i}")
            flow_manager_args.done()

        def break_folder_now(*, flow_manager_args):
            flow_manager_args.ctx.break_folder()

        def should_not_run(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("should_not_run")
            flow_manager_args.done()

        def after_folder(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("after_folder")
            flow_manager_args.done()

        def tail(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("tail")
            flow_manager_args.done()

        repeat_step = RepeatStep(
            step_id="repeat_branch",
            name="repeat_branch",
            count=4,
            children=[
                Step(step_id="inc", name="inc", func=inc_counter),
                ConditionStep(
                    step_id="if",
                    group_id="branch",
                    name="If",
                    condition_type="If",
                    condition="i == 1",
                    children=[Step(step_id="mark_if", name="mark_if", func=mark_if)],
                ),
                ConditionStep(
                    step_id="elseif",
                    group_id="branch",
                    name="ElseIf",
                    condition_type="ElseIf",
                    condition="i == 2",
                    children=[
                        Step(step_id="mark_elseif", name="mark_elseif", func=mark_elseif),
                        Step(step_id="break_folder", name="break_folder", func=break_folder_now),
                    ],
                ),
                ConditionStep(
                    step_id="else",
                    group_id="branch",
                    name="Else",
                    condition_type="Else",
                    children=[Step(step_id="mark_else", name="mark_else", func=mark_else)],
                ),
                Step(step_id="after_branch", name="after_branch", func=should_not_run),
            ],
        )

        root = Step(
            step_id="root2",
            name="root2",
            children=[
                Step(step_id="init", name="init", func=init_counter),
                FolderStep(
                    step_id="folder2",
                    name="folder2",
                    children=[
                        repeat_step,
                        Step(step_id="after_folder", name="after_folder", func=after_folder),
                    ],
                ),
                Step(step_id="tail", name="tail", func=tail),
            ],
        )

        root.execute(ctx)

        self.assertEqual(ctx.trace, ["if:1", "should_not_run", "elseif:2", "tail"])
        self.assertNotIn("else:3", ctx.trace)
        self.assertNotIn("after_folder", ctx.trace)

    def test_variable_expression_is_re_evaluated_each_repeat(self):
        Step = STEP_MOD.Step
        RepeatStep = STEP_MOD.RepeatStep

        ctx = DummyExecutionContext()

        def init_counter(*, flow_manager_args):
            flow_manager_args.ctx.update_local_variables({"i": 0})
            flow_manager_args.done()

        def inc_counter(*, flow_manager_args):
            i = flow_manager_args.ctx.variables["local"]["i"] + 1
            flow_manager_args.ctx.update_local_variables({"i": i})
            flow_manager_args.done()

        def record_x(*, flow_manager_args):
            x = flow_manager_args.ctx.variables["local"]["x"]
            flow_manager_args.ctx.trace.append(f"x:{x}")
            flow_manager_args.done()

        root = Step(
            step_id="root3",
            name="root3",
            children=[
                Step(step_id="init", name="init", func=init_counter),
                RepeatStep(
                    step_id="repeat_x",
                    name="repeat_x",
                    count=3,
                    children=[
                        Step(step_id="inc", name="inc", func=inc_counter),
                        Step(
                            step_id="set_x",
                            name="set_x",
                            variable={"x": "i + 1"},
                            func=record_x,
                        ),
                    ],
                ),
            ],
        )

        root.execute(ctx)
        self.assertEqual(ctx.trace, ["x:2", "x:3", "x:4"])

    def test_callable_condition_is_executed(self):
        Step = STEP_MOD.Step
        ConditionStep = STEP_MOD.ConditionStep

        ctx = DummyExecutionContext()

        def mark(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("callable-branch")
            flow_manager_args.done()

        root = Step(
            step_id="root4",
            name="root4",
            children=[
                ConditionStep(
                    step_id="cond_callable",
                    group_id="callable_group",
                    name="If",
                    condition_type="If",
                    condition=lambda: True,
                    children=[Step(step_id="mark", name="mark", func=mark)],
                ),
            ],
        )

        root.execute(ctx)
        self.assertEqual(ctx.trace, ["callable-branch"])

    def test_event_sub_task_step_emits_request_and_waits_in_sync_mode(self):
        Step = STEP_MOD.Step
        CallEventStep = STEP_MOD.CallEventStep

        ctx = DummyExecutionContext()
        event_tree = Step(step_id="event_tree_1", name="event_tree_1")
        ctx.state_dict["event_sub_tree_list"] = [event_tree.to_dict()]

        root = Step(
            step_id="root5",
            name="root5",
            children=[
                CallEventStep(
                    step_id="event_call_1",
                    name="event_call_1",
                    index=0,
                    run_mode="SYNC",
                ),
            ],
        )

        root.execute(ctx)

        self.assertEqual(len(ctx.event_requests), 1)
        self.assertEqual(ctx.event_requests[0]["event_task_id"], "event_tree_1")
        self.assertEqual(ctx.event_requests[0]["run_mode"], "SYNC")
        self.assertEqual(ctx.pause_calls, [])

    def test_call_event_step_reads_tree_from_state_dict_by_index(self):
        Step = STEP_MOD.Step
        CallEventStep = STEP_MOD.CallEventStep

        ctx = DummyExecutionContext()
        ctx.state_dict["event_sub_tree_list"] = [
            {"stepId": "event_tree_2", "name": "event_tree_2", "children": []}
        ]

        root = Step(
            step_id="root6",
            name="root6",
            children=[
                CallEventStep(
                    step_id="event_call_2",
                    name="event_call_2",
                    index=0,
                    run_mode="ASYNC",
                ),
            ],
        )

        root.execute(ctx)

        self.assertEqual(len(ctx.event_requests), 1)
        self.assertEqual(ctx.event_requests[0]["event_task_id"], "event_tree_2")
        self.assertEqual(ctx.event_requests[0]["run_mode"], "ASYNC")
        self.assertEqual(ctx.pause_calls, [])

    def test_sync_step_roundtrip_preserves_sync_type_for_event_tree(self):
        Step = STEP_MOD.Step
        SyncStep = STEP_MOD.SyncStep

        sync = SyncStep(
            step_id="sync_1",
            name="Sync",
            args={"flag": "f1", "timeout": None},
        )

        tree = Step(
            step_id="event_root",
            name="event_root",
            children=[sync],
        )

        rebuilt = Step.from_dict(tree.to_dict())
        self.assertEqual(len(rebuilt.children), 1)
        self.assertEqual(type(rebuilt.children[0]).__name__, "SyncStep")
        self.assertEqual(getattr(rebuilt.children[0], "flag", None), "f1")

    def test_jump_loop_keeps_condition_chain_consistent(self):
        Step = STEP_MOD.Step
        ConditionStep = STEP_MOD.ConditionStep
        JumpToStep = STEP_MOD.JumpToStep

        ctx = DummyExecutionContext()

        def init_test(*, flow_manager_args):
            flow_manager_args.ctx.update_local_variables({"test": 1})
            flow_manager_args.done()

        def plus_one(*, flow_manager_args):
            v = int(flow_manager_args.ctx.variables["local"].get("test", 0)) + 1
            flow_manager_args.ctx.update_local_variables({"test": v})
            flow_manager_args.ctx.trace.append(f"test:{v}")
            flow_manager_args.done()

        def plus_two(*, flow_manager_args):
            v = int(flow_manager_args.ctx.variables["local"].get("test", 0)) + 2
            flow_manager_args.ctx.update_local_variables({"test": v})
            flow_manager_args.ctx.trace.append(f"test:{v}")
            flow_manager_args.done()

        def alarm(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("alarm")
            flow_manager_args.done()

        def tail(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("tail")
            flow_manager_args.done()

        root = Step(
            step_id="root_jump_cond",
            name="root_jump_cond",
            children=[
                Step(step_id="init", name="init", func=init_test),
                Step(step_id="loop_anchor", name="loop_anchor"),
                ConditionStep(
                    step_id="if_test_4",
                    group_id="jump_group",
                    name="If",
                    condition_type="If",
                    condition="test == 4",
                    children=[Step(step_id="alarm", name="alarm", func=alarm)],
                ),
                ConditionStep(
                    step_id="elseif_test_2",
                    group_id="jump_group",
                    name="ElseIf",
                    condition_type="ElseIf",
                    condition="test == 2",
                    children=[
                        Step(step_id="plus_two", name="plus_two", func=plus_two),
                        JumpToStep(step_id="jump_from_elseif", name="jump", target_step_id="loop_anchor"),
                    ],
                ),
                ConditionStep(
                    step_id="else_branch",
                    group_id="jump_group",
                    name="Else",
                    condition_type="Else",
                    children=[
                        Step(step_id="plus_one", name="plus_one", func=plus_one),
                        JumpToStep(step_id="jump_from_else", name="jump", target_step_id="loop_anchor"),
                    ],
                ),
                Step(step_id="tail", name="tail", func=tail),
            ],
        )

        max_jumps = 10
        pending_target: str | None = None
        for _ in range(max_jumps):
            try:
                if pending_target is None:
                    root.execute(ctx)
                else:
                    root.execute_children(ctx, target_step_id=pending_target)
                    pending_target = None
                break
            except EXC_MOD.JumpToStepException as e:
                pending_target = e.target_step_id
        else:
            self.fail("jump loop did not converge")

        self.assertEqual(ctx.trace, ["test:2", "test:4", "alarm", "tail"])

    def test_jump_with_disabled_steps_keeps_scope_and_hits_alarm(self):
        Step = STEP_MOD.Step
        ConditionStep = STEP_MOD.ConditionStep
        JumpToStep = STEP_MOD.JumpToStep

        ctx = DummyExecutionContext()

        def init_test(*, flow_manager_args):
            flow_manager_args.ctx.update_local_variables({"test": 1})
            flow_manager_args.done()

        def plus_one(*, flow_manager_args):
            v = int(flow_manager_args.ctx.variables["local"].get("test", 0)) + 1
            flow_manager_args.ctx.update_local_variables({"test": v})
            flow_manager_args.done()

        def plus_two(*, flow_manager_args):
            v = int(flow_manager_args.ctx.variables["local"].get("test", 0)) + 2
            flow_manager_args.ctx.update_local_variables({"test": v})
            flow_manager_args.done()

        def alarm(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("alarm")
            flow_manager_args.done()

        root = Step(
            step_id="root_jump_disabled",
            name="root_jump_disabled",
            children=[
                Step(step_id="disabled_1", name="disabled_1", disabled=True),
                Step(step_id="disabled_2", name="disabled_2", disabled=True),
                Step(step_id="init", name="init", func=init_test),
                Step(step_id="anchor", name="anchor"),
                ConditionStep(
                    step_id="if_test_4",
                    group_id="jump_group2",
                    name="If",
                    condition_type="If",
                    condition="test == 4",
                    children=[Step(step_id="alarm", name="alarm", func=alarm)],
                ),
                ConditionStep(
                    step_id="elseif_test_2",
                    group_id="jump_group2",
                    name="ElseIf",
                    condition_type="ElseIf",
                    condition="test == 2",
                    children=[
                        Step(step_id="plus_two", name="plus_two", func=plus_two),
                        JumpToStep(step_id="jump_from_elseif", name="jump", target_step_id="anchor"),
                    ],
                ),
                ConditionStep(
                    step_id="else_branch",
                    group_id="jump_group2",
                    name="Else",
                    condition_type="Else",
                    children=[
                        Step(step_id="plus_one", name="plus_one", func=plus_one),
                        JumpToStep(step_id="jump_from_else", name="jump", target_step_id="anchor"),
                    ],
                ),
            ],
        )

        max_jumps = 10
        pending_target: str | None = None
        for _ in range(max_jumps):
            try:
                if pending_target is None:
                    root.execute(ctx)
                else:
                    root.execute_children(ctx, target_step_id=pending_target)
                    pending_target = None
                break
            except EXC_MOD.JumpToStepException as e:
                pending_target = e.target_step_id
        else:
            self.fail("jump loop did not converge with disabled steps")

        self.assertEqual(ctx.trace, ["alarm"])

    def test_switch_case_jump_then_if_alarm(self):
        Step = STEP_MOD.Step
        ConditionStep = STEP_MOD.ConditionStep
        JumpToStep = STEP_MOD.JumpToStep

        ctx = DummyExecutionContext()

        def init_test(*, flow_manager_args):
            flow_manager_args.ctx.update_local_variables({"test": 1})
            flow_manager_args.done()

        def plus_one(*, flow_manager_args):
            v = int(flow_manager_args.ctx.variables["local"].get("test", 0)) + 1
            flow_manager_args.ctx.update_local_variables({"test": v})
            flow_manager_args.done()

        def plus_two(*, flow_manager_args):
            v = int(flow_manager_args.ctx.variables["local"].get("test", 0)) + 2
            flow_manager_args.ctx.update_local_variables({"test": v})
            flow_manager_args.done()

        def alarm_case(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("alarm_case")
            flow_manager_args.done()

        def alarm_if(*, flow_manager_args):
            flow_manager_args.ctx.trace.append("alarm_if")
            flow_manager_args.done()

        root = Step(
            step_id="root_switch_jump",
            name="root_switch_jump",
            children=[
                Step(step_id="init", name="init", func=init_test),
                Step(step_id="anchor", name="anchor"),
                Step(
                    step_id="switch",
                    name="Switch",
                    args={"switch_value": "test"},
                    children=[
                        ConditionStep(
                            step_id="case_4",
                            group_id="switch_group",
                            name="Case",
                            condition_type="Case",
                            condition="$parent.switch_value == 4",
                            children=[Step(step_id="alarm_case", name="alarm_case", func=alarm_case)],
                        ),
                        ConditionStep(
                            step_id="case_2",
                            group_id="switch_group",
                            name="Case",
                            condition_type="Case",
                            condition="$parent.switch_value == 2",
                            children=[
                                Step(step_id="plus_two", name="plus_two", func=plus_two),
                                JumpToStep(
                                    step_id="jump_from_case_2",
                                    name="jump",
                                    target_step_id="anchor",
                                ),
                            ],
                        ),
                        ConditionStep(
                            step_id="case_default",
                            group_id="switch_group",
                            name="Case",
                            condition_type="Case",
                            condition=True,
                            children=[
                                Step(step_id="plus_one", name="plus_one", func=plus_one),
                                JumpToStep(
                                    step_id="jump_from_default",
                                    name="jump",
                                    target_step_id="anchor",
                                ),
                            ],
                        ),
                    ],
                ),
                ConditionStep(
                    step_id="if_after_switch",
                    group_id="if_group",
                    name="If",
                    condition_type="If",
                    condition="test == 4",
                    children=[Step(step_id="alarm_if", name="alarm_if", func=alarm_if)],
                ),
            ],
        )

        max_jumps = 10
        pending_target: str | None = None
        for _ in range(max_jumps):
            try:
                if pending_target is None:
                    root.execute(ctx)
                else:
                    root.execute_children(ctx, target_step_id=pending_target)
                    pending_target = None
                break
            except EXC_MOD.JumpToStepException as e:
                pending_target = e.target_step_id
        else:
            self.fail("switch/case jump loop did not converge")

        self.assertEqual(ctx.variables["local"]["test"], 4)
        self.assertEqual(ctx.trace, ["alarm_case", "alarm_if"])
        self.assertEqual(ctx.current_depth, 0)

    def test_subtask_insert_halt_is_caught_and_step_completes(self):
        Step = STEP_MOD.Step
        SubTaskStep = STEP_MOD.SubTaskStep

        ctx = DummyExecutionContext()

        def halt_here(*, flow_manager_args):
            flow_manager_args.ctx.halt_sub_task()

        sub_task_tree = Step(
            step_id="sub_task_root",
            name="sub_task_root",
            children=[Step(step_id="halt_child", name="halt_child", func=halt_here)],
        )

        subtask_step = SubTaskStep(
            step_id="subtask_insert",
            name="Load Sub Task",
            sub_task_type="INSERT",
            sub_task_script_path="/tmp/unused.py",
        )
        subtask_step._load_sub_task_trees = lambda: (sub_task_tree, None)  # type: ignore[attr-defined]

        root = Step(
            step_id="root_subtask_halt",
            name="root_subtask_halt",
            children=[subtask_step],
        )

        root.execute(ctx)

        self.assertEqual(ctx.subtask_events, [("start", "sub_task_root"), ("done", "sub_task_root")])
        self.assertIn(("done", "subtask_insert"), ctx.events)


if __name__ == "__main__":
    unittest.main()
