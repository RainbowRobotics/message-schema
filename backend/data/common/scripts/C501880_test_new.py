from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.step import RepeatStep, ConditionStep, FolderStep, Step, JumpToStep, BreakStep, SubTaskStep
from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller


tree = Step(
    step_id='69770b2223120889b310eb2f',
    name='C501880_test_new',
    children=[
        Step(
            step_id='6977393f882a5839d1bfdd98',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='697881ee3f6b7cc354f7d1c5',
            name='MoveLB',
            disabled=True,
            func_name='rb_manipulate_sdk.move.call_move_lb_clr',
            args={
                'robot_model': 'C501880',
                'move_type': 'LB',
                'orientation': 0,
             },
            post_func_name='rb_manipulate_sdk.move.call_move_lb_run',
            children=[
                Step(
                    step_id='697881ee3f6b7cc354f7d1c6',
                    name='Point',
                    disabled=True,
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
            step_id='697881ee3f6b7cc354f7d1c7',
            name='MoveLB',
            disabled=True,
            func_name='rb_manipulate_sdk.move.call_move_lb_clr',
            args={
                'robot_model': 'C501880',
                'move_type': 'LB',
                'orientation': 0,
             },
            post_func_name='rb_manipulate_sdk.move.call_move_lb_run',
            children=[
                Step(
                    step_id='697881ee3f6b7cc354f7d1c8',
                    name='Point',
                    disabled=True,
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
            step_id='6977393f882a5839d1bfdd99',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977393f882a5839d1bfdd9a',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977393f882a5839d1bfdd9b',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977393f882a5839d1bfdd9c',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977393f882a5839d1bfdd9d',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977393f882a5839d1bfdd9e',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977393f882a5839d1bfdd9f',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450270',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450271',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450272',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450273',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450274',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450275',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450276',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450277',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450278',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d9450279',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d945027a',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d945027b',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69784c44a6323fe6d945027c',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='69772c44ad30694b3c694af7',
            name='Alarm',
            disabled=True,
            func_name='rb_manipulate_sdk.alarm_or_halt',
            args={
                'title': 'awdadw',
                'content': '',
                'option': 'ALARM',
                'is_only_at_ui': False,
                'save_log': False,
             },
        ),
        Step(
            step_id='6977376061667e580b9994e4',
            name='Alarm',
            disabled=True,
            func_name='rb_manipulate_sdk.alarm_or_halt',
            args={
                'title': 'awdawd',
                'content': '',
                'option': 'ALARM',
                'is_only_at_ui': False,
                'save_log': False,
             },
        ),
        Step(
            step_id='69770b3c23120889b310eb32',
            name='Wait',
            disabled=True,
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': '3',
                'mode': 'GENERAL',
                'condition': '',
             },
        ),
        Step(
            step_id='6977136df9093dc716ee6541',
            name='Wait',
            disabled=True,
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977136df9093dc716ee6542',
            name='Wait',
            disabled=True,
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977136df9093dc716ee6543',
            name='Wait',
            disabled=True,
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977136df9093dc716ee6545',
            name='Wait',
            disabled=True,
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977136df9093dc716ee6546',
            name='Wait',
            disabled=True,
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977136df9093dc716ee6547',
            name='Wait',
            disabled=True,
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': 1,
             },
        ),
        Step(
            step_id='6977136df9093dc716ee6548',
            name='Wait',
            disabled=True,
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
        process_id='C501880_test_new',
        step=tree,
        repeat_count=1,
        robot_model='C501880',
        category='manipulate',
        post_tree=None,
    )

