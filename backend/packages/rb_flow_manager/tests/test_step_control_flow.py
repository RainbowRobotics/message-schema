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

    def enter_folder(self):
        self._folder_depth += 1

    def leave_folder(self):
        self._folder_depth -= 1

    def break_folder(self):
        if self._folder_depth > 0:
            raise EXC_MOD.BreakFolder()


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


if __name__ == "__main__":
    unittest.main()
