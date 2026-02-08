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


class ControllerExceptionPolicyTest(unittest.TestCase):
    def test_zenoh_controller_no_raise_e_pattern(self):
        text = ZENOH_CONTROLLER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("raise e", text, "Use bare raise to preserve traceback context")


if __name__ == "__main__":
    unittest.main()
