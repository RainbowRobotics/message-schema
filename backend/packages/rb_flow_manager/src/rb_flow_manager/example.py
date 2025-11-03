# 사용 예제
import time

from .controller.zenoh_controller import Zenoh_Controller
from .executor import ScriptExecutor
from .step import RepeatStep, Step

if __name__ == "__main__":
    zenoh_controller = Zenoh_Controller()

    # 트리 구조 정의 (모두 Step으로 구성)
    root = Step(
        step_id="1",
        name="root",
        children=[
            RepeatStep(
                step_id="2",
                name="Repeat",
                count=2,
                children=[
                    Step(
                        step_id="3",
                        name="MoveJ",
                        func_name="rb_manupulate_sdk.move_j",
                        children=[],
                        args={"robot_model": "C500920", "joints": "[0, 0, 0, 90, 0, 0, 0]"},
                    ),
                    Step(
                        step_id="4",
                        name="MoveJ2",
                        func_name="rb_manupulate_sdk.move_j",
                        children=[],
                        args={"robot_model": "C500920", "joints": "[0, 0, 0, 0, 0, 0, 0]"},
                    ),
                ],
            )
        ],
    )

    # 실행기 생성
    executor = ScriptExecutor(controller=zenoh_controller)

    # 여러 스크립트 동시 실행
    executor.start("script_1", root, repeat_count=2)

    # 상태 확인
    print("\n=== All States ===")
    print(executor.get_all_states())

    time.sleep(4)
    executor.pause("script_1")

    time.sleep(4)

    executor.resume("script_1")

    executor.wait_all()

    print("All scripts completed!")
