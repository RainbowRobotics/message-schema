# SLAMNAV FlatBuffer Schema Reference

이 문서는 `schemas/slam/v1/` 디렉토리의 FlatBuffer 스키마를 정리한 것입니다.

---

## 1. slamnav_control.fbs - 제어 명령

로봇의 하드웨어 및 안전 기능을 제어하는 스키마입니다.

### 1.1 Safety Field (안전 필드)

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Safety_Field` | 안전 필드 조회 요청 | `id` |
| `Response_Get_Safety_Field` | 안전 필드 조회 응답 | `id`, `safety_field: int`, `result`, `message` |
| `Request_Set_Safety_Field` | 안전 필드 설정 요청 | `id`, `safety_field: int` |
| `Response_Set_Safety_Field` | 안전 필드 설정 응답 | `id`, `safety_field: int`, `result`, `message` |

### 1.2 Safety Flag (안전 플래그)

```fbs
struct SafetyFlag {
    name: string;
    value: bool;
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Safety_Flag` | 안전 플래그 조회 요청 | `id` |
| `Response_Get_Safety_Flag` | 안전 플래그 조회 응답 | `id`, `safety_flag: [SafetyFlag]`, `result`, `message` |
| `Request_Set_Safety_Flag` | 안전 플래그 설정 요청 | `id`, `reset_flag: [SafetyFlag]` |
| `Response_Set_Safety_Flag` | 안전 플래그 설정 응답 | `id`, `reset_flag: [SafetyFlag]`, `result`, `message` |

### 1.3 Safety IO (안전 I/O)

MCU의 디지털 I/O 제어 (각 8개 채널)

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Safety_Io` | Safety I/O 조회 | `id` |
| `Response_Get_Safety_Io` | Safety I/O 응답 | `id`, `mcu0_dio[8]`, `mcu1_dio[8]`, `mcu0_din[8]`, `mcu1_din[8]`, `result`, `message` |
| `Request_Set_Safety_Io` | Safety I/O 설정 | `id`, `command`, `mcu0_din[8]`, `mcu1_din[8]` |
| `Response_Set_Safety_Io` | Safety I/O 설정 응답 | `id`, `command`, `mcu0_din[8]`, `mcu1_din[8]`, `result`, `message` |

### 1.4 Dock (도킹/충전)

**command 값:** `"dock"`, `"undock"`, `"dockstop"`

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Dock` | 도킹 명령 요청 | `id`, `command` |
| `Response_Dock` | 도킹 명령 응답 | `id`, `command`, `result`, `message` |
| `Request_Charge_Trigger` | 충전 트리거 요청 | `id`, `onoff: bool` |
| `Response_Charge_Trigger` | 충전 트리거 응답 | `id`, `onoff: bool`, `result`, `message` |
| `State_Change_Dock` | 도킹 상태 변경 알림 | `id`, `command`, `result`, `message` |

### 1.5 Obstacle Box (장애물 박스)

```fbs
struct ObsBox {
    x: float;
    y: float;
    z: float;
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Obs_Box` | 장애물 박스 조회 | `id`, `command`, `min: ObsBox`, `max: ObsBox`, `range: float` |
| `Response_Get_Obs_Box` | 장애물 박스 응답 | `id`, `command`, `min`, `max`, `range`, `result`, `message` |
| `Request_Set_Obs_Box` | 장애물 박스 설정 | `id`, `command`, `min: ObsBox`, `max: ObsBox`, `range: float` |
| `Response_Set_Obs_Box` | 장애물 박스 응답 | `id`, `command`, `min`, `max`, `range`, `result`, `message` |

### 1.6 LED 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Led` | LED 제어 요청 | `id`, `onoff: bool`, `color: string` |
| `Response_Led` | LED 제어 응답 | `id`, `onoff`, `color`, `result`, `message` |

### 1.7 Motor 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Motor` | 모터 On/Off 요청 | `id`, `onoff: bool` |
| `Response_Motor` | 모터 On/Off 응답 | `id`, `onoff`, `result`, `message` |

### 1.8 Jog 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Jog` | Jog 모드 On/Off 요청 | `id`, `onoff: bool` |
| `Response_Jog` | Jog 모드 On/Off 응답 | `id`, `onoff`, `result`, `message` |

### 1.9 Sensor 제어

센서 소켓 on/off 및 주파수 설정
**command 값:** `"camera"`, `"lidar2d"`, `"lidar3d"`

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Sensor` | 센서 제어 요청 | `id`, `command`, `onoff: bool`, `frequency: int` |
| `Response_Sensor` | 센서 제어 응답 | `id`, `command`, `onoff`, `frequency`, `result`, `message` |

### 1.10 Path 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Path` | 경로 제어 요청 | `id`, `onoff: bool`, `frequency: int` |
| `Response_Path` | 경로 제어 응답 | `id`, `onoff`, `frequency`, `result`, `message` |

### 1.11 Detect (마커 감지)

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Detect` | 마커 감지 요청 | `id`, `command`, `camera_number: int`, `camera_serial`, `marker_size: float` |
| `Response_Detect` | 마커 감지 응답 | `id`, `command`, `camera_number`, `camera_serial`, `marker_size`, `tf: [float]` (4x4 Matrix), `result`, `message` |

### 1.12 Control_Result (공통 응답)

```fbs
table Control_Result {
    id: string;
    result: string;   // "success" / "fail"
    message: string;  // reason
}
```

---

## 2. slamnav_localization.fbs - 위치 추정

로봇의 위치 추정(Localization)을 제어하는 스키마입니다.

### 2.1 LocalizationPose 구조체

```fbs
struct LocalizationPose {
    x: float;
    y: float;
    z: float;
    rz: float;
}
```

### 2.2 초기화 명령

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Localization_Init` | 수동 초기화 요청 | `id`, `pose: LocalizationPose` |
| `Response_Localization_Init` | 수동 초기화 응답 | `id`, `pose`, `result`, `message` |
| `Request_Localization_RandomInit` | 랜덤 초기화 요청 | `id`, `random_seed: string` |
| `Response_Localization_RandomInit` | 랜덤 초기화 응답 | `id`, `random_seed`, `pose`, `result`, `message` |
| `Request_Localization_AutoInit` | 자동 초기화 요청 | `id` |
| `Response_Localization_AutoInit` | 자동 초기화 응답 | `id`, `pose`, `result`, `message` |
| `Request_Localization_SemiAutoInit` | 반자동 초기화 요청 | `id` |
| `Response_Localization_SemiAutoInit` | 반자동 초기화 응답 | `id`, `pose`, `result`, `message` |

### 2.3 시작/정지 명령

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Localization_Start` | 위치 추정 시작 | `id` |
| `Response_Localization_Start` | 위치 추정 시작 응답 | `id`, `pose`, `result`, `message` |
| `Request_Localization_Stop` | 위치 추정 정지 | `id` |
| `Response_Localization_Stop` | 위치 추정 정지 응답 | `id`, `pose`, `result`, `message` |

---

## 3. slamnav_map.fbs - 맵 관리

맵 파일 및 토폴로지 관리를 위한 스키마입니다.

### 3.1 맵 파일 정보

```fbs
table MapFileInfo {
    map_name: string;
    created_at: string;
    update_at: string;
    map_type: string;
    map_size: float;
}

table MapFile {
    file_name: string;
    created_at: string;
    update_at: string;
    file_type: string;
    cloud_info: MapFileInfo;
    topo_info: MapFileInfo;
}
```

### 3.2 맵 목록 관리

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Map_List` | 맵 목록 조회 | `id` |
| `Response_Map_List` | 맵 목록 응답 | `id`, `list: [MapFile]`, `result`, `message` |
| `Request_Map_Delete` | 맵 삭제 요청 | `id`, `map_name` |
| `Response_Map_Delete` | 맵 삭제 응답 | `id`, `map_name`, `result`, `message` |
| `Request_Map_Current` | 현재 맵 조회 | `id` |
| `Response_Map_Current` | 현재 맵 응답 | `id`, `map_name`, `result`, `message` |
| `Request_Map_Load` | 맵 로드 요청 | `id`, `map_name` |
| `Response_Map_Load` | 맵 로드 응답 | `id`, `map_name`, `result`, `message` |

### 3.3 Cloud Data (포인트 클라우드)

```fbs
struct CloudData {
    x: float;
    y: float;
    z: float;
    intensity: float;
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Map_File` | 맵 파일 조회 | `id`, `map_name`, `file_name` |
| `Response_Get_Map_File` | 맵 파일 응답 | `id`, `map_name`, `file_name`, `size`, `data: [CloudData]`, `result`, `message` |
| `Request_Get_Map_Cloud` | 맵 클라우드 조회 | `id`, `map_name`, `file_name` |
| `Response_Get_Map_Cloud` | 맵 클라우드 응답 | `id`, `map_name`, `file_name`, `size`, `data: [CloudData]`, `result`, `message` |
| `Request_Set_Map_Cloud` | 맵 클라우드 설정 | `id`, `map_name`, `file_name`, `size`, `data: [CloudData]` |
| `Response_Set_Map_Cloud` | 맵 클라우드 응답 | `id`, `map_name`, `file_name`, `size`, `result`, `message` |

### 3.4 Topology (노드/링크)

```fbs
table Link {
    id: string;
    info: string;
    speed: float;
    method: string;
    safety_field: int;
}

table Node {
    id: string;
    name: string;
    pose: [float];
    info: string;
    links: [Link];
    type: string;
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Map_Topology` | 토폴로지 조회 | `id`, `map_name` |
| `Response_Get_Map_Topology` | 토폴로지 응답 | `id`, `map_name`, `data: [Node]`, `result`, `message` |
| `Request_Set_Map_Topology` | 토폴로지 설정 | `id`, `map_name`, `data: [Node]` |
| `Response_Set_Map_Topology` | 토폴로지 설정 응답 | `id`, `map_name`, `result`, `message` |

### 3.5 Mapping (맵 생성)

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Mapping_Start` | 맵핑 시작 | `id` |
| `Response_Mapping_Start` | 맵핑 시작 응답 | `id`, `result`, `message` |
| `Request_Mapping_Stop` | 맵핑 정지 | `id` |
| `Response_Mapping_Stop` | 맵핑 정지 응답 | `id`, `result`, `message` |
| `Request_Mapping_Save` | 맵 저장 | `id`, `map_name` |
| `Response_Mapping_Save` | 맵 저장 응답 | `id`, `map_name`, `result`, `message` |

---

## 4. slamnav_move.fbs - 이동 제어

로봇의 이동을 제어하는 스키마입니다.

### 4.1 MovePose 구조체

```fbs
struct MovePose {
    x: float;
    y: float;
    z: float;
    rz: float;
}
```

### 4.2 이동 명령

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Move_Goal` | 목표 노드로 이동 | `id`, `goal_id`, `goal_name`, `method`, `preset: int` |
| `Response_Move_Goal` | 목표 노드 이동 응답 | `id`, `goal_id`, `goal_name`, `method`, `preset`, `result`, `message` |
| `Request_Move_Target` | 목표 좌표로 이동 | `id`, `goal_pose: MovePose`, `method`, `preset: int` |
| `Response_Move_Target` | 목표 좌표 이동 응답 | `id`, `goal_pose`, `method`, `preset`, `result`, `message` |

### 4.3 이동 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Move_Stop` | 이동 정지 | `id` |
| `Response_Move_Stop` | 이동 정지 응답 | `id`, `result`, `message` |
| `Request_Move_Pause` | 이동 일시정지 | `id` |
| `Response_Move_Pause` | 이동 일시정지 응답 | `id`, `result`, `message` |
| `Request_Move_Resume` | 이동 재개 | `id` |
| `Response_Move_Resume` | 이동 재개 응답 | `id`, `result`, `message` |

### 4.4 특수 이동

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Move_XLinear` | X축 직선 이동 | `id`, `target: float`, `speed: float` |
| `Response_Move_XLinear` | X축 직선 이동 응답 | `id`, `target`, `speed`, `result`, `message` |
| `Request_Move_Circular` | 원형 이동 | `id`, `target: float`, `speed: float`, `direction: string` |
| `Response_Move_Circular` | 원형 이동 응답 | `id`, `target`, `speed`, `direction`, `result`, `message` |
| `Request_Move_Rotate` | 회전 이동 | `id`, `target: float`, `speed: float` |
| `Response_Move_Rotate` | 회전 이동 응답 | `id`, `target`, `speed`, `result`, `message` |

### 4.5 Jog 명령

```fbs
table Move_Jog {
    vx: float;
    vy: float;
    wz: float;
}
```

### 4.6 State_Change_Move (이동 상태 변경 알림)

```fbs
table State_Change_Move {
    id: string;
    command: string;
    cur_pose: MovePose;
    goal_pose: MovePose;
    map_name: string;
    vel: [float];
    goal_id: string;
    goal_name: string;
    method: string;
    direction: string;
    preset: int;
    result: string;
    message: string;
    remaining_dist: float;
    target: float;
    speed: float;
    bat_percent: int;
}
```

---

## 5. slamnav_multi.fbs - 멀티 로봇

다중 로봇 환경에서의 경로 및 가상 장애물 관리입니다.

### 5.1 경로 공유

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Path` | 경로 요청 | `id`, `path: [string]` |
| `Response_Path` | 경로 응답 | `id`, `path: [string]`, `result`, `message` |

### 5.2 VOBS (가상 장애물)

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Vobs` | VOBS 요청 | `id`, `vobs_robots: [string]`, `vobs_closures: [string]`, `is_vobs_closures_change: string` |
| `Response_Vobs` | VOBS 응답 | `id`, `vobs_robots`, `vobs_closures`, `is_vobs_closures_change`, `result`, `message` |

---

## 6. slamnav_setting.fbs - 설정

로봇의 설정 파라미터 관리 스키마입니다.

### 6.1 Robot Type

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Robot_Type` | 로봇 타입 조회 | `id` |
| `Response_Get_Robot_Type` | 로봇 타입 응답 | `id`, `robot_type`, `result`, `message` |
| `Request_Set_Robot_Type` | 로봇 타입 설정 | `id`, `robot_type` |
| `Response_Set_Robot_Type` | 로봇 타입 설정 응답 | `id`, `robot_type`, `result`, `message` |

### 6.2 Setting Param

```fbs
table SettingParam {
    key: string;
    type: string;
    value: string;
}
```

### 6.3 PDU Param

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Pdu_Param` | PDU 파라미터 조회 | `id` |
| `Response_Get_Pdu_Param` | PDU 파라미터 응답 | `id`, `params: [SettingParam]`, `result`, `message` |
| `Request_Set_Pdu_Param` | PDU 파라미터 설정 | `id`, `params: [SettingParam]` |
| `Response_Set_Pdu_Param` | PDU 파라미터 설정 응답 | `id`, `params`, `result`, `message` |

### 6.4 Drive Param

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Drive_Param` | 드라이브 파라미터 조회 | `id` |
| `Response_Get_Drive_Param` | 드라이브 파라미터 응답 | `id`, `params: [SettingParam]`, `result`, `message` |

### 6.5 Sensor Index

```fbs
table SensorInfo {
    index: int;
    serial: string;
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Get_Sensor_Index` | 센서 인덱스 조회 | `id`, `target: string` |
| `Response_Get_Sensor_Index` | 센서 인덱스 응답 | `id`, `target`, `index: [SensorInfo]`, `result`, `message` |
| `Request_Set_Sensor_Index` | 센서 인덱스 설정 | `id`, `target`, `index: [SensorInfo]` |
| `Response_Set_Sensor_Index` | 센서 인덱스 설정 응답 | `id`, `target`, `index`, `result`, `message` |
| `Request_Set_Sensor_On` | 센서 On 요청 | `id`, `index: [SensorInfo]` |
| `Response_Set_Sensor_On` | 센서 On 응답 | `id`, `index`, `result`, `message` |
| `Request_Get_Sensor_Off` | 센서 Off 조회 | `id`, `index: [SensorInfo]` |
| `Response_Get_Sensor_Off` | 센서 Off 응답 | `id`, `index`, `result`, `message` |

---

## 7. slamnav_socket.fbs - 센서 데이터 스트림

실시간 센서 데이터 스트리밍을 위한 스키마입니다.

### 7.1 Point 구조체

```fbs
struct Point2D {
    x: float;
    y: float;
}

struct Point3D {
    x: float;
    y: float;
    z: float;
}
```

### 7.2 Lidar 데이터

| Table | 설명 | 필드 |
|-------|------|------|
| `Lidar_2D` | 2D 라이다 데이터 | `timestamp: float`, `points: [Point2D]` |
| `Lidar_3D` | 3D 라이다 데이터 | `timestamp: float`, `points: [Point3D]` |
| `Mapping_Cloud` | 맵핑 클라우드 데이터 | `timestamp: float`, `points: [Point3D]` |

### 7.3 Path 데이터

| Table | 설명 | 필드 |
|-------|------|------|
| `Global_Path` | 글로벌 경로 | `id`, `path: [string]` |
| `Local_Path` | 로컬 경로 | `id`, `path: [string]` |

---

## 8. slamnav_status.fbs - 상태 정보

로봇의 실시간 상태 정보를 제공하는 스키마입니다.

### 8.1 IMU 상태

```fbs
struct StatusImu {
    acc_x: float;
    acc_y: float;
    acc_z: float;
    gyr_x: float;
    gyr_y: float;
    gyr_z: float;
    imu_rx: float;
    imu_ry: float;
    imu_rz: float;
}
```

### 8.2 Motor 상태

```fbs
struct StatusMotor {
    connection: bool;
    status: int;
    temp: float;
    current: float;
}
```

### 8.3 Condition 상태

```fbs
struct StatusCondition {
    inlier_error: float;
    inlier_ratio: float;
    mapping_error: float;
    mapping_ratio: float;
}
```

### 8.4 Robot State

```fbs
table StatusRobotState {
    charge: string;
    dock: bool;
    emo: bool;
    localization: string;
    power: bool;
    sss_recovery: bool;
    sw_reset: bool;
    sw_stop: bool;
    sw_start: bool;
    sf_bumper_detect: bool;
    sf_obs_detect: bool;
    sf_operational_stop: bool;
}
```

### 8.5 Safety IO State

```fbs
table StatusRobotSafetyIoState {
    mcu0_dio: [bool];   // 8개
    mcu1_dio: [bool];   // 8개
    mcu0_din: [bool];   // 8개
    mcu1_din: [bool];   // 8개
}
```

### 8.6 Power 상태

```fbs
struct StatusPower {
    bat_in: float;
    bat_out: float;
    bat_current: float;
    total_power: float;
    power: float;
    bat_percent: float;
    tabos_voltage: float;
    tabos_current: float;
    tabos_status: float;
    tabos_ttf: float;
    tabos_tte: float;
    tabos_soc: float;
    tabos_soh: float;
    tabos_temp: float;
    tabos_rc: float;
    tabos_ae: float;
    charge_current: float;
    contact_voltage: float;
}
```

### 8.7 Setting/Map 상태

```fbs
table StatusSetting {
    platform_type: string;
    platform_name: string;
}

table StatusMap {
    map_name: string;
    map_status: string;
}
```

### 8.8 종합 Status

```fbs
table Status {
    condition: StatusCondition;
    imu: StatusImu;
    motor0: StatusMotor;
    motor1: StatusMotor;
    power: StatusPower;
    robot_state: StatusRobotState;
    robot_safety_io_state: StatusRobotSafetyIoState;
    setting: StatusSetting;
    map: StatusMap;
}
```

### 8.9 Move Status (이동 상태)

```fbs
table MoveStatusMoveState {
    auto_move: string;
    dock_move: string;
    jog_move: string;
    obs: string;
    path: string;
}

struct MoveStatusPose {
    x: float;
    y: float;
    z: float;
    rz: float;
}

struct MoveStatusVel {
    vx: float;
    vy: float;
    wz: float;
}

table MoveStatusNode {
    node_id: string;
    name: string;
    state: string;
    x: float;
    y: float;
    rz: float;
}

table Move_Status {
    cur_node: MoveStatusNode;
    goal_node: MoveStatusNode;
    move_state: MoveStatusMoveState;
    pose: MoveStatusPose;
    vel: MoveStatusVel;
}
```

---

## 9. slamnav_update.fbs - 업데이트

소프트웨어 업데이트 관련 스키마입니다.

| Table | 설명 | 필드 |
|-------|------|------|
| `Request_Update` | 업데이트 요청 | `id`, `branch`, `version` |
| `Response_Update` | 업데이트 응답 | `id`, `branch`, `version`, `result`, `message` |
| `Request_Current_Version` | 현재 버전 조회 | `id` |
| `Response_Current_Version` | 현재 버전 응답 | `id`, `version`, `result`, `message` |

---

## 공통 패턴

### Request/Response 패턴

모든 명령은 Request/Response 패턴을 따릅니다:
- **Request**: `id` 필드 포함 (요청 식별자)
- **Response**: `id`, `result`, `message` 필드 포함
  - `result`: `"success"` 또는 `"fail"`
  - `message`: 실패 시 사유

### Result 테이블

각 도메인별 공통 Result 테이블이 정의되어 있습니다:
- `Control_Result`
- `Localization_Result`
- `Map_Result`
- `Move_Result`
- `Setting_Result`
- `Status_Result`
- `Update_Result`

---

## 파일 매핑

| FBS 파일 | C++ 구현 파일 |
|----------|--------------|
| slamnav_control.fbs | comm_zenoh_control.cpp |
| slamnav_localization.fbs | comm_zenoh_localization.cpp |
| slamnav_map.fbs | comm_zenoh_map.cpp |
| slamnav_move.fbs | comm_zenoh_move.cpp |
| slamnav_multi.fbs | comm_zenoh_path.cpp |
| slamnav_setting.fbs | comm_zenoh_setting.cpp |
| slamnav_socket.fbs | comm_zenoh_sensor.cpp |
| slamnav_status.fbs | comm_zenoh_status.cpp, comm_zenoh_moveStatus.cpp |
| slamnav_update.fbs | comm_zenoh_update.cpp |
