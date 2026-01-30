from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.step import RepeatStep, ConditionStep, FolderStep, Step, JumpToStep, BreakStep, SubTaskStep
from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller


tree = Step(
    step_id='697731010eb1e8f43fbe9a0d',
    name='TEST1_TEST_PLS',
    children=[

    ],
)


if __name__ == "__main__":

    zenoh_controller = Zenoh_Controller()
    executor = ScriptExecutor(controller=zenoh_controller)

    executor.start(
        process_id='TEST1_TEST_PLS',
        step=tree,
        repeat_count=1,
        robot_model='TEST1',
        category='manipulate',
        post_tree=None,
    )

