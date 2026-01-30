from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.step import RepeatStep, ConditionStep, FolderStep, Step, JumpToStep, BreakStep, SubTaskStep
from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller


tree = Step(
    step_id='69770b2223120889b310eb2e',
    name='C500880_test_new',
    children=[
        Step(
            step_id='6979d57fd1a0155cbef53d90',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d91',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d92',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d93',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d94',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d95',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d96',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d97',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d98',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d99',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d9a',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d9b',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6979d57fd1a0155cbef53d9c',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
    ],
)


if __name__ == "__main__":

    zenoh_controller = Zenoh_Controller()
    executor = ScriptExecutor(controller=zenoh_controller)

    executor.start(
        process_id='C500880_test_new',
        step=tree,
        repeat_count=1,
        robot_model='C500880',
        category='manipulate',
        post_tree=None,
    )

