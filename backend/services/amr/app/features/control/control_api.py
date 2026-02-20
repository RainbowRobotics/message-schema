"""
[AMR 이동 API 어댑터]
"""

from fastapi import APIRouter

from .control_schema import (
    RequestControlChargeTriggerPD,
    RequestControlDetectMarkerPD,
    RequestControlJogModePD,
    RequestControlLedModePD,
    RequestControlMotorModePD,
    RequestControlSetObsBoxPD,
    RequestSetSafetyFieldPD,
    RequestSetSafetyFlagPD,
    RequestSetSafetyIoPD,
    ResponseControlChargeTriggerPD,
    ResponseControlDetectMarkerPD,
      ResponseControlDockPD,
    ResponseControlGetObsBoxPD,
    ResponseControlJogModePD,
    ResponseControlLedModePD,
    ResponseControlMotorModePD,
    ResponseControlSetObsBoxPD,
    ResponseGetSafetyFieldPD,
    ResponseGetSafetyFlagPD,
    ResponseGetSafetyIoPD,
    ResponseSetSafetyFieldPD,
    ResponseSetSafetyFlagPD,
    ResponseSetSafetyIoPD,
)
from .control_service import AmrControlService

amr_control_router = APIRouter(
    tags=["AMR 제어"],
    prefix="",
)

amr_control_service = AmrControlService()

@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/dock",
    summary="도킹 요청",
    description= """
도킹 명령을 요청합니다.

## 📌 기능 설명
- 도킹스테이션을 지원하는 모델만 사용가능합니다.
- dock : 도킹 시작 명령
  - 도킹 명령은 위치초기화, 맵 로드와 상관없이 가능합니다.
  - 도킹 완료 후에는 반드시 언도킹을 수행한 뒤에 로봇이동이 가능합니다. 그렇지 않으면 스테이션과 로봇이 충돌나거나 장애물 인식으로 인해 움직이지 않을 수 있습니다.
  - 도킹 명령을 시작하기 위해서는 로봇의 카메라가 도킹스테이션을 바라보고 있는 방향으로 약 1m 가량 떨어져있어야 합니다.
  - 위의 조건을 만족하기 위해, 도킹스테이션을 고정된 자리에 두고 이를 바라보는 방향으로 로봇의 노드를 맵 상에 추가하는 것을 권장합니다.
  - 도킹 시작 시, 인식된 도킹스테이션 방향으로 로봇이 이동합니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="도킹 처리 결과 반환"
)
async def slamnav_control_dock(robot_model: str, robot_id: str) -> ResponseControlDockPD:
    return await amr_control_service.control_dock(robot_model, robot_id)

@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/undock",
    summary="도킹 해제 요청",
    description= """
도킹 해제 명령을 요청합니다.

## 📌 기능 설명
- 도킹스테이션을 지원하는 모델만 사용가능합니다.
- undock : 도킹 해제 명령
  - 도킹 완료 후에는 반드시 언도킹을 수행한 뒤에 로봇이동이 가능합니다. 그렇지 않으면 스테이션과 로봇이 충돌나거나 장애물 인식으로 인해 움직이지 않을 수 있습니다.
  - 도킹 중이 아닌데 언도킹을 하면 뒤로 1m 가량 이동할 수 있습니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="도킹 해제 처리 결과 반환"
)
async def slamnav_control_undock(robot_model: str, robot_id: str) -> ResponseControlDockPD:
    return await amr_control_service.control_undock(robot_model, robot_id)

@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/dockStop",
    summary="도킹 종료 요청",
    description= """
도킹 종료 명령을 요청합니다.

## 📌 기능 설명
- 도킹스테이션을 지원하는 모델만 사용가능합니다.
- dockStop : 도킹 종료 명령
  - 도킹 시퀀스를 종료시키고 로봇이 이동정지합니다.


## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="도킹 처리 결과 반환"
)
async def slamnav_control_dock_stop(robot_model: str, robot_id: str) -> ResponseControlDockPD:
    return await amr_control_service.control_dock_stop(robot_model, robot_id)

@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/chargeTrigger",
    summary="충전 트리거 요청",
    description= """
충전 트리거 명령을 요청합니다.

## 📌 기능 설명
- 도킹스테이션을 지원하는 모델만 사용가능합니다.
- 일부 도킹스테이션 모델의 경우 충전 완료 후 도킹스테이션이 꺼집니다.
- 충전 완료 이후 로봇이 계속 켜져있으면 전원이 방전되므로 일정 조건 도달 시, 충전 트리거를 요청해주세요.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| control | bool | ✅ | - | 충전 트리거 켜기/끄기 | True/False |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| control | bool | 충전 트리거 켜기/끄기 | True/False |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="충전 트리거 처리 결과 반환"
)
async def slamnav_control_chargeTrigger(robot_model: str, robot_id: str, request: RequestControlChargeTriggerPD) -> ResponseControlChargeTriggerPD:
    return await amr_control_service.control_chargeTrigger(robot_model, robot_id, request)


@amr_control_router.get(
    "/{robot_model}/{robot_id}/control/getSafetyField",
    summary="세이프티 영역 조회",
    description= """
현재 설정된 세이프티 영역을 조회합니다.

## 📌 기능 설명
- 세이프티영역을 지원하는 모델만 사용가능합니다.
- 세이프티영역은 라이다 센서의 장애물 인식 영역을 설정하는 기능입니다.
- 세이프티영역은 id값으로 지정되며 id값에 대한 영역설정은 라이다에서 설정가능합니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| safetyField | int | 설정된 세이프티 영역 | 1 |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="세이프티 영역 조회 결과 반환"
)
async def slamnav_control_getSafetyField(robot_model: str, robot_id: str) -> ResponseGetSafetyFieldPD:
    return await amr_control_service.control_get_safetyField(robot_model, robot_id)


@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/setSafetyField",
    summary="세이프티 영역 설정",
    description= """
세이프티 영역을 설정합니다.

## 📌 기능 설명
- 세이프티영역을 지원하는 모델만 사용가능합니다.
- 세이프티영역은 라이다 센서의 장애물 인식 영역을 설정하는 기능입니다.
- 세이프티영역은 id값으로 지정되며 id값에 대한 영역설정은 라이다에서 설정가능합니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| safetyField | int | ✅ | - | 세이프티 영역 | 1 |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| safetyField | int | 설정된 세이프티 영역 | 1 |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_setSafetyField(robot_model: str, robot_id: str, request: RequestSetSafetyFieldPD) -> ResponseSetSafetyFieldPD:
    return await amr_control_service.control_set_safetyField(robot_model, robot_id, request)



@amr_control_router.get(
    "/{robot_model}/{robot_id}/control/getSafetyFlag",
    summary="세이프티 플래그 조회",
    description= """
세이프티 플래그를 조회합니다.

## 📌 기능 설명
- 세이프티 기능을 지원하는 모델만 사용가능합니다.
- 세이프티기능으로 트리거된 플래그를 조회할 수 있습니다.
- 초기화 가능한 기능은 아래와 같습니다. (버전에 따라 변경될 수 있음)
  - bumper : 범퍼 감지(충돌)로 멈춤
  - interlock : 로봇Arm등의 상위 동작으로 인해 멈춤
  - obstacle : 장애물 감지로 멈춤
  - operationStop : 장애물 감지 등 세이프티 기능으로 멈춘상태

## 📌 SafetyFlag

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| name | string | ✅ | - | 세이프티 플래그 이름 | 'obstacle', 'interlock', 'bumper', 'operationStop' |
| value | bool | ✅ | - | 세이프티 플래그 값 | false |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| safetyFlag | SafetyFlag[] | 세이프티 플래그 | [{name:'obstacle',value:false}, {name:'interlock',value:true}] |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_getSafetyFlag(robot_model: str, robot_id: str) -> ResponseGetSafetyFlagPD:
    return await amr_control_service.control_get_safetyFlag(robot_model, robot_id)



@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/setSafetyFlag",
    summary="세이프티 플래그 설정",
    description= """
세이프티 플래그를 설정합니다.

## 📌 기능 설명
- 세이프티 기능을 지원하는 모델만 사용가능합니다.
- 세이프티기능으로 트리거된 플래그를 초기화할 수 있습니다.
- 값은 기본적으로 false로 초기화만 가능합니다.
- 초기화 가능한 기능은 아래와 같습니다. (버전에 따라 변경될 수 있음)
  - bumper : 범퍼 감지(충돌)로 멈춤
  - interlock : 로봇Arm등의 상위 동작으로 인해 멈춤
  - obstacle : 장애물 감지로 멈춤
  - operationStop : 명령에 의한 멈춤

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| safetyFlag | SafetyFlag[] | ✅ | - | 세이프티 플래그 초기화 | [{name:'obstacle',value:false}] |

## 📌 SafetyFlag

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| name | string | ✅ | - | 세이프티 플래그 이름 | 'obstacle', 'interlock', 'bumper', 'operationStop' |
| value | bool | ✅ | - | 세이프티 플래그 값 | false |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| safetyFlag | SafetyFlag[] | 세이프티 플래그 | [{name:'obstacle',value:false}] |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_setSafetyFlag(robot_model: str, robot_id: str, request: RequestSetSafetyFlagPD) -> ResponseSetSafetyFlagPD:
    return await amr_control_service.control_set_safetyFlag(robot_model, robot_id, request)



@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/led",
    summary="LED 수동 제어",
    description= """
LED의 수동 제어를 요청합니다.

## 📌 기능 설명
- onoff값이 true인 경우 LED를 수동으로 제어합니다. LED의 색상을 설정하거나 켜고 끌 수 있습니다. 이때 SLAMNAV 내부 시스템에 의한 제어는 무시됩니다.
- onoff값이 false인 경우 SLAMNAV 내부 시스템에 의해 LED가 자동으로 제어됩니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| control | bool | ✅ | - | LED 수동 제어 켜고 끌지를 결정합니다. | true |
| color | string | - | - | LED 색상을 입력합니다. onoff가 true일 경우에만 사용됩니다. | 'none','red', 'blue', 'white', 'green', 'magenta', 'yellow', 'red_blink', 'blue_blink', 'white_blink', 'green_blink', 'magenta_blink', 'yellow_blink' |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| control | bool | LED 수동 제어 켜고 끌지를 결정합니다. | true |
| color | string | LED 색상을 입력합니다. onoff가 true일 경우에만 사용됩니다. | 'none','red', 'blue', 'white', 'green', 'magenta', 'yellow', 'red_blink', 'blue_blink', 'white_blink', 'green_blink', 'magenta_blink', 'yellow_blink' |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
- SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_led(robot_model: str, robot_id: str, request: RequestControlLedModePD) -> ResponseControlLedModePD:
    return await amr_control_service.control_led(robot_model, robot_id, request)


@amr_control_router.get(
    "/{robot_model}/{robot_id}/control/getSafetyIo",
    summary="세이프티 IO 조회",
    description= """
세이프티 IO를 조회합니다.

## 📌 기능 설명
- 세이프티 IO를 지원하는 모델만 사용가능합니다.
- 세이프티 IO는 MCU의 DIO, DIN을 조회하는 기능입니다.
- 세이프티 IO는 0번 핀부터 7번 핀까지 순서대로 조회합니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| mcu0Dio | bool[] | MCU0 DIO. 0번 핀부터 7번 핀 순서대로 조회합니다. | [False, True, False, True, False, True, False, True] |
| mcu0Din | bool[] | MCU0 DIN. 0번 핀부터 7번 핀 순서대로 조회합니다. | [False, True, False, True, False, True, False, True] |
| mcu1Dio | bool[] | MCU1 DIO. 0번 핀부터 7번 핀 순서대로 조회합니다. | [False, True, False, True, False, True, False, True] |
| mcu1Din | bool[] | MCU1 DIN. 0번 핀부터 7번 핀 순서대로 조회합니다. | [False, True, False, True, False, True, False, True] |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_getSafetyIo(robot_model: str, robot_id: str) -> ResponseGetSafetyIoPD:
    return await amr_control_service.control_get_safetyIo(robot_model, robot_id)

@amr_control_router.post(
    "/{robot_model}/control/setSafetyIo",
    summary="세이프티 IO 설정",
    description= """
세이프티 IO를 설정합니다.

## 📌 기능 설명
- 세이프티 IO를 지원하는 모델만 사용가능합니다.
- MCU의 DIO를 제어하는 기능입니다.
- MCU의 DIO는 0번 핀부터 7번 핀까지 순서대로 제어합니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| mcu0Din | bool[] | ✅ | - | MCU0 Digital Input (8bit) | [False, True, False, True, False, True, False, True] |
| mcu1Din | bool[] | ✅ | - | MCU1 Digital Input (8bit) | [False, True, False, True, False, True, False, True] |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| mcu0Din | bool[] | MCU0 Digital Input (8bit) | [False, True, False, True, False, True, False, True] |
| mcu1Din | bool[] | MCU1 Digital Input (8bit) | [False, True, False, True, False, True, False, True] |
| mcu0Dio | bool[] | MCU0 Digital Output (8bit) | [False, True, False, True, False, True, False, True] |
| mcu1Dio | bool[] | MCU1 Digital Output (8bit) | [False, True, False, True, False, True, False, True] |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_set_safetyIo(robot_model: str, robot_id: str, request: RequestSetSafetyIoPD) -> ResponseSetSafetyIoPD:
    return await amr_control_service.control_set_safetyIo(robot_model, robot_id, request)

@amr_control_router.post(
  "/{robot_model}/{robot_id}/control/motor",
  summary="모터 제어",
  description= """
모터 제어를 요청합니다.

## 📌 기능 설명
- 모터 제어를 요청합니다.
- 기능이 꺼져있을 경우 모터 입력으로 주행하지 않습니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| switch | bool | ✅ | - | 모터 사용여부 | true |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| switch | bool | 모터 사용여부 | true |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_set_motor(robot_model: str, robot_id: str, request: RequestControlMotorModePD) -> ResponseControlMotorModePD:
    return await amr_control_service.control_set_motor(robot_model, robot_id, request)

@amr_control_router.post(
  "/{robot_model}/{robot_id}/control/jog",
  summary="조이스틱 사용여부 설정",
  description= """
조이스틱 사용여부를 설정합니다.

## 📌 기능 설명
- 조이스틱 사용여부를 설정합니다.
- 기능이 꺼져있을 경우 조이스틱 입력으로 주행하지 않습니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| switch | bool | ✅ | - | 조이스틱 사용여부 | true |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| switch | bool | 조이스틱 사용여부 | true |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_set_jog(robot_model: str, robot_id: str, request: RequestControlJogModePD) -> ResponseControlJogModePD:
    return await amr_control_service.control_set_jog(robot_model, robot_id, request)

@amr_control_router.get(
    "/{robot_model}/{robot_id}/control/getObsBox",
    summary="장애물감지 영역 조회",
    description= """
장애물감지 영역을 조회합니다.

## 📌 기능 설명

- 장애물감지 영역을 지원하는 모델만 사용가능합니다.
- 장애물감지 영역은 AMR 상부의 Torso, Arm이 움직일때 장애물감지 영역을 추가로 설정하기 위해 사용됩니다.

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| minX | number | 장애물감지 영역 최소 x값 | 1.3 |
| maxX | number | 장애물감지 영역 최대 x값 | 1.3 |
| minY | number | 장애물감지 영역 최소 y값 | 1.3 |
| maxY | number | 장애물감지 영역 최대 y값 | 1.3 |
| minZ | number | 장애물감지 영역 최소 z값 | 1.3 |
| maxZ | number | 장애물감지 영역 최대 z값 | 1.3 |
| mapRange | number | 장애물감지 영역 맵 범위 | 1.3 |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스

### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_getObsBox(robot_model: str, robot_id: str) -> ResponseControlGetObsBoxPD:
    return await amr_control_service.control_get_obsbox(robot_model, robot_id)


@amr_control_router.post(
    "/{robot_model}/{robot_id}/control/setObsBox",
    summary="장애물감지 영역 설정",
    description= """
장애물감지 영역을 설정합니다.

## 📌 기능 설명
- 장애물감지 영역을 지원하는 모델만 사용가능합니다.
- 장애물감지 영역은 AMR 상부의 Torso, Arm이 움직일때 장애물감지 영역을 추가로 설정하기 위해 사용됩니다.

## 📌 요청 바디(JSON)

| 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
|-|-|-|-|-|-|
| minX | number | ✅ | m | 장애물감지 영역 최소 x값 | 1.3 |
| maxX | number | ✅ | m | 장애물감지 영역 최대 x값 | 1.3 |
| minY | number | ✅ | m | 장애물감지 영역 최소 y값 | 1.3 |
| maxY | number | ✅ | m | 장애물감지 영역 최대 y값 | 1.3 |
| minZ | number | ✅ | m | 장애물감지 영역 최소 z값 | 1.3 |
| maxZ | number | ✅ | m | 장애물감지 영역 최대 z값 | 1.3 |
| mapRange | number | ✅ | m | 장애물감지 영역 맵 범위 | 1.3 |

## 📌 응답 바디(JSON)

| 필드명       | 타입    | 설명                          | 예시 |
|-------------|---------|-------------------------------|--------|
| minX | number | 장애물감지 영역 최소 x값 | 1.3 |
| maxX | number | 장애물감지 영역 최대 x값 | 1.3 |
| minY | number | 장애물감지 영역 최소 y값 | 1.3 |
| maxY | number | 장애물감지 영역 최대 y값 | 1.3 |
| minZ | number | 장애물감지 영역 최소 z값 | 1.3 |
| maxZ | number | 장애물감지 영역 최대 z값 | 1.3 |
| mapRange | number | 장애물감지 영역 맵 범위 | 1.3 |
| result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
| message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

## ⚠️ 에러 케이스
### **403** INVALID_ARGUMENT
  - 파라메터가 없거나 잘못된 값일 때
### **409** CONFLICT
  - 요청한 명령을 수행할 수 없을 때
  - SLAMNAV에서 거절했을 때
### **500** INTERNAL_SERVER_ERROR
  - DB관련 에러 등 서버 내부적인 에러
### **502** BAD_GATEWAY
  - SLAMNAV와 연결되지 않았을 때
### **504** DEADLINE_EXCEEDED
  - SLAMNAV로부터 응답을 받지 못했을 때
    """,
    response_description="특정 작업 처리 결과 반환"
)
async def slamnav_control_setObsBox(robot_model: str, robot_id: str, request: RequestControlSetObsBoxPD) -> ResponseControlSetObsBoxPD:
    return await amr_control_service.control_set_obsbox(robot_model, robot_id, request)


# @amr_control_router.post(
#     "/{robot_model}/{robot_id}/control/detectMarker",
#     summary="마커 감지 요청",
#     description= """
#   마커를 감지합니다.

#   ## 📌 기능 설명
#   - 카메라 등의 센서로 감지를 요청합니다.
#   - 아르코마커 인식 후 마커의 pose(x,y,z,roll,pitch,yaw)와 tf(4x4)를 조회합니다.
#   - 로봇에 부착된 시리얼넘버는 모델마다 상이하며 번호로는 전면(0번), 오른쪽(1번), 왼쪽(2번), 후면(3번) 카메라가 존재합니다.
#   - 아르코마커의 사이즈를 m단위로 입력해야합니다.
#   - cameraNumber와 cameraSerial 값이 없으면 내부 로직에 의해 인식 후 반환합니다.

#   ## 📌 요청 바디(JSON)

#   | 필드명 | 타입 | 필수 | 단위 | 설명 | 예시 |
#   |-|-|-|-|-|-|
#   | cameraNumber | number | - | - | 카메라 번호 | 0 |
#   | cameraSerial | string | - | - | 카메라 시리얼넘버 | '1234567890' |
#   | markerSize | number | - | m | 아르코마커 사이즈 | 0.1 |

#   ## 📌 응답 바디(JSON)

#   | 필드명       | 타입    | 설명                          | 예시 |
#   |-------------|---------|-------------------------------|--------|
#   | command | string | 감지 명령 | 'aruco' |
#   | cameraNumber | number | 카메라 번호 | 0 |
#   | cameraSerial | string | 카메라 시리얼넘버 | '1234567890' |
#   | markerSize | number | 아르코마커 사이즈 | 0.1 |
#   | pose | number[] | 아르코마커의 pose(x,y,z,roll,pitch,yaw) | [[0.1, -2.3, 0.0, 0.0, 0.0, 0.0]] |
#   | tf | number[] | 아르코마커의 tf(4x4) | [[1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, -2.3, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]] |
#   | result | string | 요청한 명령에 대한 결과입니다. | 'accept', 'reject' |
#   | message | string | result값이 reject 인 경우 SLAMNAV에서 보내는 메시지 입니다. | '' |

#   ## ⚠️ 에러 케이스
#   ### **403** INVALID_ARGUMENT
#   - 파라메터가 없거나 잘못된 값일 때
#   ### **409** CONFLICT
#   - 요청한 명령을 수행할 수 없을 때
#   - SLAMNAV에서 거절했을 때
#   ### **500** INTERNAL_SERVER_ERROR
#   - DB관련 에러 등 서버 내부적인 에러
#   ### **502** BAD_GATEWAY
#   - SLAMNAV와 연결되지 않았을 때
#   ### **504** DEADLINE_EXCEEDED
#   - SLAMNAV로부터 응답을 받지 못했을 때
#     """,
#     response_description="센서 감지 처리 결과 반환"
# )
# async def slamnav_control_detectMarker(robot_model: str, robot_id: str, request: RequestControlDetectMarkerPD) -> ResponseControlDetectMarkerPD:
#     return await amr_control_service.control_detectMarker(robot_model, robot_id, request)
