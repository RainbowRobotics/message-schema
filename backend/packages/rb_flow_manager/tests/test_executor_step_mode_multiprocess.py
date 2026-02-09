import sys
import time
import unittest
from pathlib import Path

PACKAGES_DIR = Path(__file__).resolve().parents[2]
SRC_PATHS = [
    PACKAGES_DIR / "rb_flow_manager" / "src",
    PACKAGES_DIR / "rb_utils" / "src",
    PACKAGES_DIR / "rb_schemas" / "src",
    PACKAGES_DIR / "rb_sdk" / "src",
    PACKAGES_DIR / "rb_flat_buffers" / "src",
]
for src_path in SRC_PATHS:
    src = str(src_path)
    if src not in sys.path:
        sys.path.insert(0, src)

from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.schema import RB_Flow_Manager_ProgramState
from rb_flow_manager.step import Step, SyncStep


def _make_step_tree(tag: str) -> Step:
    # root(1st step) -> leaf(2nd step, step_mode barrier expected)
    return Step(
        step_id=f"{tag}_root",
        name=f"{tag}_root",
        children=[
            Step(step_id=f"{tag}_leaf", name=f"{tag}_leaf"),
        ],
    )

def _make_step_tree_with_sync(tag: str, sync_flag: str) -> Step:
    return Step(
        step_id=f"{tag}_root",
        name=f"{tag}_root",
        children=[
            SyncStep(
                step_id=f"{tag}_sync",
                name=f"{tag}_sync",
                flag=sync_flag,
            ),
            Step(step_id=f"{tag}_leaf", name=f"{tag}_leaf"),
        ],
    )

class StepModeMultiProcessTest(unittest.TestCase):
    def test_step_mode_with_4_main_and_3_sub_each_completes(self):
        executor = ScriptExecutor(min_step_interval=0.0)

        main_pids = [f"main_{i}" for i in range(4)]
        batch = []

        for main_pid in main_pids:
            batch.append({
                "process_id": main_pid,
                "step": _make_step_tree(main_pid),
                "repeat_count": 1,
                "robot_model": None,
                "category": "manipulate",
                "step_mode": True,
                "min_step_interval": 0.0,
                "is_ui_execution": False,
                "post_tree": None,
                "parent_process_id": None,
            })

            for idx in range(3):
                sub_pid = f"{main_pid}_sub_{idx}"
                batch.append({
                    "process_id": sub_pid,
                    "step": _make_step_tree(sub_pid),
                    "repeat_count": 1,
                    "robot_model": None,
                    "category": "manipulate",
                    "step_mode": True,
                    "min_step_interval": 0.0,
                    "is_ui_execution": False,
                    "post_tree": None,
                    "parent_process_id": main_pid,
                })

        self.assertEqual(len(batch), 16)
        self.assertTrue(executor.start(batch))

        deadline = time.time() + 10.0
        saw_waiting = False
        while time.time() < deadline:
            states = [executor.get_state(arg["process_id"]).get("state") for arg in batch]
            if any(state == RB_Flow_Manager_ProgramState.WAITING for state in states):
                saw_waiting = True
                break
            time.sleep(0.05)

        self.assertTrue(saw_waiting, "expected at least one process to enter WAITING in step_mode")

        executor.resume_all()

        self.assertTrue(executor.wait_all(timeout=15.0))

        settle_deadline = time.time() + 3.0
        while time.time() < settle_deadline:
            states = [executor.get_state(arg["process_id"]).get("state") for arg in batch]
            if all(state != RB_Flow_Manager_ProgramState.RUNNING for state in states):
                break
            time.sleep(0.05)

        for arg in batch:
            pid = arg["process_id"]
            state_info = executor.get_state(pid)
            state = state_info.get("state")
            self.assertFalse(state_info.get("is_alive"), f"process {pid} is still alive")
            self.assertNotEqual(state, RB_Flow_Manager_ProgramState.ERROR, f"process {pid} errored")
            self.assertNotEqual(state, RB_Flow_Manager_ProgramState.STOPPED, f"process {pid} stopped")
            self.assertEqual(state, RB_Flow_Manager_ProgramState.COMPLETED, f"process {pid} did not complete")


    def test_step_mode_with_sync_step_for_4_main_and_3_sub_each(self):
        executor = ScriptExecutor(min_step_interval=0.0)

        main_pids = [f"main_sync_{i}" for i in range(4)]
        batch = []
        args_map = {}
        sync_flag = "sync_all_step_mode"

        for main_pid in main_pids:
            main_arg = {
                "process_id": main_pid,
                "step": _make_step_tree_with_sync(main_pid, sync_flag),
                "repeat_count": 1,
                "robot_model": None,
                "category": "manipulate",
                "step_mode": True,
                "min_step_interval": 0.0,
                "is_ui_execution": False,
                "post_tree": None,
                "parent_process_id": None,
            }
            batch.append(main_arg)
            args_map[main_pid] = main_arg

            for idx in range(3):
                sub_pid = f"{main_pid}_sub_{idx}"
                sub_arg = {
                    "process_id": sub_pid,
                    "step": _make_step_tree_with_sync(sub_pid, sync_flag),
                    "repeat_count": 1,
                    "robot_model": None,
                    "category": "manipulate",
                    "step_mode": True,
                    "min_step_interval": 0.0,
                    "is_ui_execution": False,
                    "post_tree": None,
                    "parent_process_id": main_pid,
                }
                batch.append(sub_arg)
                args_map[sub_pid] = sub_arg

        self.assertEqual(len(batch), 16)
        self.assertTrue(executor.start(batch))

        tick_pid = batch[0]["process_id"]
        deadline = time.time() + 20.0

        while time.time() < deadline:
            if executor.wait_all(timeout=0.05):
                break

            states = [executor.get_state(arg["process_id"]).get("state") for arg in batch]
            if all(
                s in (RB_Flow_Manager_ProgramState.WAITING, RB_Flow_Manager_ProgramState.PAUSED)
                for s in states
            ):
                self.assertTrue(executor.start(args_map[tick_pid]))
            else:
                time.sleep(0.02)

        self.assertTrue(executor.wait_all(timeout=2.0), "sync+step_mode batch did not finish in time")

        for arg in batch:
            pid = arg["process_id"]
            state_info = executor.get_state(pid)
            self.assertFalse(state_info.get("is_alive"), f"{pid} is still alive")
            self.assertNotEqual(state_info.get("state"), RB_Flow_Manager_ProgramState.ERROR, f"{pid} errored")
            self.assertNotEqual(state_info.get("state"), RB_Flow_Manager_ProgramState.STOPPED, f"{pid} stopped")



if __name__ == "__main__":
    unittest.main()
