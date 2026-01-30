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
        RepeatStep(
            step_id='697c340f01e944dcb633b2b8',
            name='Repeat',
            count=-1,
            children=[
                SyncStep(
                    step_id='697c340f01e944dcb633b2b9',
                    name='Sync',
                    args={
                        'flag': '1',
                        'timeout': None,
                     },
                ),
                Step(
                    step_id='697c340f01e944dcb633b2ba',
                    name='Wait',
                    func_name='rb_manipulate_sdk.program.manipulate_wait',
                    args={
                        'wait_type': 'TIME',
                        'second': 1,
                     },
                ),
                Step(
                    step_id='697c340f01e944dcb633b2bb',
                    name='Wait',
                    func_name='rb_manipulate_sdk.program.manipulate_wait',
                    args={
                        'wait_type': 'TIME',
                        'second': 1,
                     },
                ),
            ],
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
        Step(
            step_id='697bf99b63551189750300fe',
            name='MoveL',
            args={
                'robot_model': 'C500880',
                'move_type': 'L',
                'orientation': None,
             },
            children=[
                Step(
                    step_id='697bf99b63551189750300ff',
                    name='Point',
                    func_name='rb_manipulate_sdk.point.set_point',
                    args={
                        'robot_model': '$parent.robot_model',
                        'move_type': '$parent.move_type',
                        'input_method': 0,
                        'pnt_para': 100,
                        'pnt_type': 0,
                        'tar_frame': 0,
                        'tar_unit': 0,
                        'tar_values': [0, 0, 0, 0, 0, 0, 0],
                        'spd_mode': 0,
                        'spd_vel_para': 0.4,
                        'spd_acc_para': 0.1,
                        'tcp_num': -1,
                        'orientation': '$parent.orientation',
                     },
                ),
            ],
        ),
        Step(
            step_id='697bf99b6355118975030100',
            name='MoveJ',
            args={
                'robot_model': 'C500880',
                'move_type': 'J',
                'orientation': None,
             },
            children=[
                Step(
                    step_id='697bf99b6355118975030101',
                    name='Point',
                    func_name='rb_manipulate_sdk.point.set_point',
                    args={
                        'robot_model': '$parent.robot_model',
                        'move_type': '$parent.move_type',
                        'input_method': 0,
                        'pnt_para': 100,
                        'pnt_type': 0,
                        'tar_frame': -1,
                        'tar_unit': 0,
                        'tar_values': [0, 0, 0, 0, 0, 0, 0],
                        'spd_mode': 0,
                        'spd_vel_para': 0.4,
                        'spd_acc_para': 0.1,
                        'tcp_num': -1,
                        'orientation': '$parent.orientation',
                     },
                ),
            ],
        ),
    ],
)

post_tree = Step(
    step_id='697bf99b63551189750300fc',
    name='PostProgram',
    children=[
        Step(
            step_id='697bf99b63551189750300fd',
            name='Empty',
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
        post_tree=post_tree,
    )

