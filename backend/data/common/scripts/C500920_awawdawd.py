from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.step import RepeatStep, ConditionStep, FolderStep, Step, JumpToStep, BreakStep, SubTaskStep
from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller


tree = Step(
    step_id='69733f8069aaed490ba1cc3d',
    name='C500920_awawdawd',
    children=[
        Step(
            step_id='69733f9569aaed490ba1cc3e',
            name='MoveJB',
            args={
                'robot_model': 'C500920',
                'move_type': 'JB',
             },
            post_func_name='rb_manipulate_sdk.move.call_move_jb_run',
            children=[
                Step(
                    step_id='69733f9569aaed490ba1cc3f',
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
                        'tar_values': [0, 0, 0, 0, 90, 0, 0],
                        'spd_mode': 0,
                        'spd_vel_para': 0.4,
                        'spd_acc_para': 0.1,
                        'tcp_num': -1,
                     },
                ),
            ],
        ),
    ],
)


if __name__ == "__main__":

    zenoh_controller = Zenoh_Controller()
    executor = ScriptExecutor(controller=zenoh_controller)

    executor.start(
        process_id='C500920_awawdawd',
        step=tree,
        repeat_count=1,
        robot_model='C500920',
        category='manipulate',
        post_tree=None,
    )

