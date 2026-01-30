from rb_flow_manager.executor import ScriptExecutor
from rb_flow_manager.step import RepeatStep, ConditionStep, FolderStep, Step, JumpToStep, BreakStep, SubTaskStep
from rb_flow_manager.controller.zenoh_controller import Zenoh_Controller


tree = Step(
    step_id='69733f8069aaed490ba1cc3d',
    name='C500920_awawdawd',
    children=[
        Step(
            step_id='6976b24815e344e679327882',
            name='MoveJB',
            disabled=True,
            func_name='rb_manipulate_sdk.move.call_move_jb_clr',
            args={
                'robot_model': 'C500920',
                'move_type': 'JB',
             },
            post_func_name='rb_manipulate_sdk.move.call_move_jb_run',
            children=[
                Step(
                    step_id='6976b2b715e344e679327884',
                    name='Point',
                    disabled=True,
                    func_name='rb_manipulate_sdk.point.set_point',
                    args={
                        'robot_model': '$parent.robot_model',
                        'move_type': '$parent.move_type',
                        'input_method': 0,
                        'pnt_para': 100,
                        'pnt_type': 0,
                        'tar_frame': -1,
                        'tar_unit': 0,
                        'tar_values': [0.133, 0.897, -0.054, 0.042, 100, 0.263, -0.096],
                        'spd_mode': 0,
                        'spd_vel_para': 0.4,
                        'spd_acc_para': 0.1,
                        'tcp_num': -1,
                     },
                ),
                Step(
                    step_id='6976b24815e344e679327883',
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
                        'tar_values': [128, 0, 300, 63.893, 0, -166.241, 283.759],
                        'spd_mode': 0,
                        'spd_vel_para': 0.4,
                        'spd_acc_para': 0.1,
                        'tcp_num': -1,
                     },
                ),
                Step(
                    step_id='6976b2cc15e344e679327885',
                    name='Point',
                    disabled=True,
                    func_name='rb_manipulate_sdk.point.set_point',
                    args={
                        'robot_model': '$parent.robot_model',
                        'move_type': '$parent.move_type',
                        'input_method': 0,
                        'pnt_para': 100,
                        'pnt_type': 0,
                        'tar_frame': -1,
                        'tar_unit': 0,
                        'tar_values': [0, 0, 0, 0, 80, 0, 0],
                        'spd_mode': 0,
                        'spd_vel_para': 0.4,
                        'spd_acc_para': 0.1,
                        'tcp_num': -1,
                     },
                ),
            ],
        ),
        Step(
            step_id='697735786d6f0e05000b5669',
            name='Alarm',
            func_name='rb_manipulate_sdk.program.alarm_or_halt',
            args={
                'title': '',
                'content': '',
                'option': 'ALARM',
                'is_only_at_ui': False,
                'save_log': False,
             },
        ),
        Step(
            step_id='6976ca33a78d3c4dcc47a6ae',
            name='ToolOut',
            func_name='rb_manipulate_sdk.program.tool_out',
            args={
                'tool_dout_args': [{'port_num': 0, 'desired_out': -1}, {'port_num': 1, 'desired_out': 0}],
                'tool_voltage': 12,
             },
            memo='',
        ),
        Step(
            step_id='6975ac8921e984616cb7951b',
            name='Variables',
            func_name='rb_base_sdk.set_variables',
            args={
                'variables': [{'type': 'STRING', 'name': 'asd', 'init_value': '"asdasdasd"'}],
             },
        ),
        Step(
            step_id='6976ca33a78d3c4dcc47a6af',
            name='Script',
            func_name='rb_base_sdk.make_script',
            args={
                'mode': 'GENERAL',
                'general_contents': [],
                'contents': '"""\nScript Step Guide\n\nGet variable:\n  value = variables["name"]\n  value = variables.get("name", None)\n\nUpdate variable:\n  update_variable({"name": value})\n\nLogging:\n  rb_log.info("message")\n  rb_log.debug("message", disable_db=True) # disable_db is optional, default is False\n\nExecution control:\n  done() # finish the script step\n  pause() # pause the script step\n  stop() # stop the script step\n  check_stop() # check if the script step is stopped\n"""',
             },
        ),
        Step(
            step_id='6975c6dd9ca52536a6c06438',
            name='Alarm',
            func_name='rb_manipulate_sdk.alarm_or_halt',
            args={
                'title': 'asd',
                'content': '',
                'option': 'ALARM',
                'is_only_at_ui': False,
                'save_log': False,
             },
        ),
        Step(
            step_id='6975ac8921e984616cb7951d',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': '3',
                'mode': 'GENERAL',
                'condition': '',
             },
        ),
        Step(
            step_id='6976ca33a78d3c4dcc47a6b0',
            name='PinJoint',
            func_name='rb_manipulate_sdk.program.set_pin_point_or_joint',
            args={
                'input_method': 0,
                'pin_type': 'JOINT',
                'target': {'tar_frame': 0, 'tar_unit': 0, 'tar_values': [0, 0, 0, 0, 0, 0, 0]},
             },
        ),
        Step(
            step_id='6975ad66af3406abdcac0c1c',
            name='Wait',
            func_name='rb_manipulate_sdk.program.manipulate_wait',
            args={
                'wait_type': 'TIME',
                'second': '10',
                'mode': 'GENERAL',
                'condition': '',
             },
        ),
        Step(
            step_id='6975ad66af3406abdcac0c1d',
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
        process_id='C500920_awawdawd',
        step=tree,
        repeat_count=1,
        robot_model='C500920',
        category='manipulate',
        post_tree=None,
    )

