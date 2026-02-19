import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "rb_flow_manager"
EXECUTOR_PATH = ROOT / "executor.py"
ZENOH_CONTROLLER_PATH = ROOT / "controller" / "zenoh_controller.py"


def _parse(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


class ExecutorContractTest(unittest.TestCase):
    def test_sub_task_done_callback_uses_three_arguments(self):
        module = _parse(EXECUTOR_PATH)

        for node in ast.walk(module):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "_on_sub_task_done":
                self.assertEqual(
                    len(node.args),
                    3,
                    "on_sub_task_done callback must receive pid, sub_task_id, sub_task_type",
                )
                return

        self.fail("_on_sub_task_done callback invocation not found")

    def test_done_callback_uses_pid_and_step_id(self):
        module = _parse(EXECUTOR_PATH)

        for node in ast.walk(module):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "_on_done":
                self.assertEqual(
                    len(node.args),
                    2,
                    "on_done callback must receive pid and step_id",
                )
                return

        self.fail("_on_done callback invocation not found")

    def test_handle_event_validates_required_fields(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn("done event requires step_id", text)
        self.assertIn("sub_task_done event requires sub_task_id and valid sub_task_type", text)
        self.assertIn("sub_task_start event requires sub_task_id and valid sub_task_type", text)
        self.assertIn("event_sub_task_start event requires event_task_id", text)
        self.assertIn("event_sub_task_start event requires event_tree", text)

    def test_waiting_pause_does_not_propagate_to_parent_or_children(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn("if not is_wait:", text)
        self.assertIn("if parent_process_id is not None:", text)
        self.assertIn("self.pause(parent_process_id)", text)

    def test_collect_sync_flags_ignores_disabled_steps(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn('if getattr(s, "disabled", False):', text)

    def test_monitor_stops_children_when_parent_finishes(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn("def _stop_children_of_finished_parents(self):", text)
        self.assertIn("self._stop_children_of_finished_parents()", text)

    def test_batch_round_wait_and_group_progress_logic_exists(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn('"type": "round_wait"', text)
        self.assertIn("def _progress_batch_rounds(self):", text)
        self.assertIn("self._progress_batch_rounds()", text)
        self.assertIn("def _spawn_group_children_for_next_round(self, group_id: str):", text)

    def test_child_process_repeat_count_is_forced_to_one(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn("if parent_process_id is not None:", text)
        self.assertIn("repeat_count = 1", text)

    def test_sub_task_repeats_execute_root_not_children_only(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn("elif state_dict[\"current_repeat\"] > 1 and not is_sub_task:", text)

    def test_ignore_pause_is_reset_in_executor_paths(self):
        text = EXECUTOR_PATH.read_text(encoding="utf-8")
        self.assertIn('self.state_dicts[process_id]["ignore_pause"] = False', text)
        self.assertIn('self.state_dicts[pid]["ignore_pause"] = False', text)
        self.assertIn("if self.state_dicts[process_id].get(\"ignore_pause\"):", text)
        self.assertIn("self.pause_events[process_id].clear()", text)

class ControllerExceptionPolicyTest(unittest.TestCase):
    def test_zenoh_controller_no_raise_e_pattern(self):
        text = ZENOH_CONTROLLER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("raise e", text, "Use bare raise to preserve traceback context")


if __name__ == "__main__":
    unittest.main()
