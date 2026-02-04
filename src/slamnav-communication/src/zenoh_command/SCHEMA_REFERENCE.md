# SLAMNAV FlatBuffer Schema Reference

이 문서는 `schemas/amr/v1/` 디렉토리의 FlatBuffer 스키마를 정리한 것입니다.

---

## 1. slamnav_control.fbs - 제어 명령

로봇의 하드웨어 및 안전 기능을 제어하는 스키마입니다.

### 1.1 Safety Field (안전 필드)

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetSafetyField` | 안전 필드 조회 요청 | `id` |
| `ResponseGetSafetyField` | 안전 필드 조회 응답 | `id`, `safety_field: int`, `result`, `message` |
| `RequestSetSafetyField` | 안전 필드 설정 요청 | `id`, `safety_field: int` |
| `ResponseSetSafetyField` | 안전 필드 설정 응답 | `id`, `safety_field: int`, `result`, `message` |

### 1.2 Safety Flag (안전 플래그)

```fbs
table SafetyFlag {
    name: string;   // obstacle, bumper, interlock, operationStop
    value: bool;
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetSafetyFlag` | 안전 플래그 조회 요청 | `id` |
| `ResponseGetSafetyFlag` | 안전 플래그 조회 응답 | `id`, `safety_flag: [SafetyFlag]`, `result`, `message` |
| `RequestSetSafetyFlag` | 안전 플래그 설정 요청 | `id`, `reset_flag: [SafetyFlag]` |
| `ResponseSetSafetyFlag` | 안전 플래그 설정 응답 | `id`, `reset_flag: [SafetyFlag]`, `result`, `message` |

### 1.3 Safety IO (안전 I/O)

MCU의 디지털 I/O 제어 (각 8개 채널)

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetSafetyIo` | Safety I/O 조회 | `id` |
| `ResponseGetSafetyIo` | Safety I/O 응답 | `id`, `mcu0_dio[8]`, `mcu1_dio[8]`, `mcu0_din[8]`, `mcu1_din[8]`, `result`, `message` |
| `RequestSetSafetyIo` | Safety I/O 설정 | `id`, `mcu0_din[8]`, `mcu1_din[8]` |
| `ResponseSetSafetyIo` | Safety I/O 설정 응답 | `id`, `mcu0_din[8]`, `mcu1_din[8]`, `result`, `message` |

### 1.4 Dock (도킹/충전)

**command 값:** `"dock"`, `"undock"`, `"dockstop"`

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestDockControl` | 도킹 명령 요청 | `id`, `command` |
| `ResponseDockControl` | 도킹 명령 응답 | `id`, `command`, `result`, `message` |
| `RequestChargeTrigger` | 충전 트리거 요청 | `id`, `control: bool` |
| `ResponseChargeTrigger` | 충전 트리거 응답 | `id`, `control: bool`, `result`, `message` |
| `ResultControlDock` | 도킹 상태 변경 알림 | `id`, `command`, `result`, `message` |

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
| `RequestGetObsBox` | 장애물 박스 조회 | `id` |
| `ResponseGetObsBox` | 장애물 박스 응답 | `id`, `min: ObsBox`, `max: ObsBox`, `range: float`, `result`, `message` |
| `RequestSetObsBox` | 장애물 박스 설정 | `id`, `min: ObsBox`, `max: ObsBox`, `range: float` |
| `ResponseSetObsBox` | 장애물 박스 응답 | `id`, `min: ObsBox`, `max: ObsBox`, `range: float`, `result`, `message` |

### 1.6 LED 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestLedMode` | LED 제어 요청 | `id`, `control: bool`, `color: string` |
| `ResponseLedMode` | LED 제어 응답 | `id`, `control`, `color`, `result`, `message` |

### 1.7 Motor 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestMotorMode` | 모터 On/Off 요청 | `id`, `control: bool` |
| `ResponseMotorMode` | 모터 On/Off 응답 | `id`, `control`, `result`, `message` |

### 1.8 Jog 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestJog` | Jog 모드 On/Off 요청 | `id`, `control: bool` |
| `ResponseJog` | Jog 모드 On/Off 응답 | `id`, `control`, `result`, `message` |

### 1.9 Sensor 제어

센서 소켓 on/off 및 주파수 설정
**command 값:** `"camera"`, `"lidar2d"`, `"lidar3d"`

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestSensorMode` | 센서 제어 요청 | `id`, `command`, `control: bool`, `frequency: int` |
| `ResponseSensorMode` | 센서 제어 응답 | `id`, `command`, `control`, `frequency`, `result`, `message` |

### 1.10 Path 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestPathMode` | 경로 제어 요청 | `id`, `control: bool`, `frequency: int` |
| `ResponsePathMode` | 경로 제어 응답 | `id`, `control`, `frequency`, `result`, `message` |

### 1.11 Detect (마커 감지)

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestDetectMarker` | 마커 감지 요청 | `id`, `number: int`, `serial: string`, `m_size: float` |
| `ResponseDetectMarker` | 마커 감지 응답 | `id`, `number`, `serial`, `m_size`, `tf: [float]` (4x4 Matrix), `result`, `message` |

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
| `RequestLocalizationInit` | 수동 초기화 요청 | `id`, `pose: LocalizationPose` |
| `ResponseLocalizationInit` | 수동 초기화 응답 | `id`, `pose`, `result`, `message` |
| `RequestLocalizationRandomInit` | 랜덤 초기화 요청 | `id` |
| `ResponseLocalizationRandomInit` | 랜덤 초기화 응답 | `id`, `pose`, `result`, `message` |
| `RequestLocalizationAutoInit` | 자동 초기화 요청 | `id` |
| `ResponseLocalizationAutoInit` | 자동 초기화 응답 | `id`, `pose`, `result`, `message` |
| `RequestLocalizationSemiAutoInit` | 반자동 초기화 요청 | `id` |
| `ResponseLocalizationSemiAutoInit` | 반자동 초기화 응답 | `id`, `pose`, `result`, `message` |

### 2.3 시작/정지 명령

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestLocalizationStart` | 위치 추정 시작 | `id` |
| `ResponseLocalizationStart` | 위치 추정 시작 응답 | `id`, `pose`, `result`, `message` |
| `RequestLocalizationStop` | 위치 추정 정지 | `id` |
| `ResponseLocalizationStop` | 위치 추정 정지 응답 | `id`, `pose`, `result`, `message` |

### 2.4 Result (결과 알림)

| Table | 설명 | 필드 |
|-------|------|------|
| `ResultLocalizationInit` | 위치 초기화 결과 응답 | `id`, `pose`, `result`, `message` |

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
| `RequestMapList` | 맵 목록 조회 | `id` |
| `ResponseMapList` | 맵 목록 응답 | `id`, `list: [MapFile]`, `result`, `message` |
| `RequestMapDelete` | 맵 삭제 요청 | `id`, `map_name` |
| `ResponseMapDelete` | 맵 삭제 응답 | `id`, `map_name`, `result`, `message` |
| `RequestMapCurrent` | 현재 맵 조회 | `id` |
| `ResponseMapCurrent` | 현재 맵 응답 | `id`, `map_name`, `result`, `message` |
| `RequestMapLoad` | 맵 로드 요청 | `id`, `map_name` |
| `ResponseMapLoad` | 맵 로드 응답 | `id`, `map_name`, `result`, `message` |
| `RequestTopoLoad` | 토폴로지 로드 요청 | `id` |
| `ResponseTopoLoad` | 토폴로지 로드 응답 | `id`, `map_name`, `result`, `message` |

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
| `RequestGetMapFile` | 맵 파일 조회 | `id`, `map_name`, `file_name` |
| `ResponseGetMapFile` | 맵 파일 응답 | `id`, `map_name`, `file_name`, `size: uint32`, `data: [CloudData]`, `result`, `message` |
| `RequestGetMapCloud` | 맵 클라우드 조회 | `id`, `map_name`, `file_name` |
| `ResponseGetMapCloud` | 맵 클라우드 응답 | `id`, `map_name`, `file_name`, `size: uint32`, `data: [CloudData]`, `result`, `message` |
| `RequestSetMapCloud` | 맵 클라우드 설정 | `id`, `map_name`, `file_name`, `size: uint32`, `data: [CloudData]` |
| `ResponseSetMapCloud` | 맵 클라우드 응답 | `id`, `map_name`, `file_name`, `result`, `message` |

### 3.4 Topology (노드/링크)

```fbs
table NodePose {
    x: float;
    y: float;
    z: float;
    roll: float;
    pitch: float;
    yaw: float;
}

table NodeSize {
    x_size: float;
    y_size: float;
    z_size: float;
}

table Link {
    id: string;
    dir: string;           // 주행 방향 ( "FORWARD" / "REVERSE" )
    method: string;        // 이동 방법 ( "PP" / "HPP" / "QD" )
    speed: float;
    safety_field: int;
}

table Node {
    context: string;
    id: string;
    links: [Link];
    name: string;
    pose: NodePose;
    role: string;
    size: NodeSize;
    type: string;          // "GOAL" / "INIT" / "ROUTE" / "STATION" / "ARUCO"
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetMapTopology` | 토폴로지 조회 | `id`, `map_name`, `file_name`, `page_no`, `page_size`, `total_page`, `node_type`, `search_text`, `sort_option`, `sort_direction` |
| `ResponseGetMapTopology` | 토폴로지 응답 | `id`, `map_name`, `file_name`, `page_no`, `page_size`, `total_page`, `node_type`, `search_text`, `sort_option`, `sort_direction`, `data: [Node]`, `result`, `message` |
| `RequestSetMapTopology` | 토폴로지 설정 | `id`, `map_name`, `data: [Node]` |
| `ResponseSetMapTopology` | 토폴로지 설정 응답 | `id`, `map_name`, `result`, `message` |

### 3.5 Mapping (맵 생성)

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestMappingStart` | 맵핑 시작 | `id` |
| `ResponseMappingStart` | 맵핑 시작 응답 | `id`, `result`, `message` |
| `RequestMappingStop` | 맵핑 정지 | `id` |
| `ResponseMappingStop` | 맵핑 정지 응답 | `id`, `result`, `message` |
| `RequestMappingSave` | 맵 저장 | `id`, `map_name` |
| `ResponseMappingSave` | 맵 저장 응답 | `id`, `map_name`, `result`, `message` |
| `RequestMappingCloudReload` | 맵핑 클라우드 리로드 | `id` |
| `ResponseMappingCloudReload` | 맵핑 클라우드 리로드 응답 | `id`, `result`, `message` |

### 3.6 Result (결과 알림)

| Table | 설명 | 필드 |
|-------|------|------|
| `ResultMapLoad` | 맵 로드 결과 응답 | `id`, `map_name`, `result`, `message` |

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
| `RequestMoveGoal` | 목표 노드로 이동 | `id`, `goal_id`, `goal_name`, `method`, `preset: int` |
| `ResponseMoveGoal` | 목표 노드 이동 응답 | `id`, `goal_id`, `goal_name`, `method`, `preset`, `result`, `message` |
| `RequestMoveTarget` | 목표 좌표로 이동 | `id`, `goal_pose: MovePose`, `method`, `preset: int` |
| `ResponseMoveTarget` | 목표 좌표 이동 응답 | `id`, `method`, `goal_pose`, `preset`, `result`, `message` |

### 4.3 이동 제어

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestMoveStop` | 이동 정지 | `id` |
| `ResponseMoveStop` | 이동 정지 응답 | `id`, `result`, `message` |
| `RequestMovePause` | 이동 일시정지 | `id` |
| `ResponseMovePause` | 이동 일시정지 응답 | `id`, `result`, `message` |
| `RequestMoveResume` | 이동 재개 | `id` |
| `ResponseMoveResume` | 이동 재개 응답 | `id`, `result`, `message` |

### 4.4 특수 이동

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestMoveXLinear` | X축 직선 이동 | `id`, `target: float`, `speed: float` |
| `ResponseMoveXLinear` | X축 직선 이동 응답 | `id`, `target`, `speed`, `result`, `message` |
| `RequestMoveYLinear` | Y축 직선 이동 | `id`, `target: float`, `speed: float` |
| `ResponseMoveYLinear` | Y축 직선 이동 응답 | `id`, `target`, `speed`, `result`, `message` |
| `RequestMoveCircular` | 원형 이동 | `id`, `target: float`, `speed: float`, `direction: string` |
| `ResponseMoveCircular` | 원형 이동 응답 | `id`, `target`, `speed`, `direction`, `result`, `message` |
| `RequestMoveRotate` | 회전 이동 | `id`, `target: float`, `speed: float` |
| `ResponseMoveRotate` | 회전 이동 응답 | `id`, `target`, `speed`, `result`, `message` |

### 4.5 Jog 명령 (Pub/Sub)

```fbs
table MoveJog {
    vx: float;
    vy: float;
    wz: float;
}
```

### 4.6 ResultMove (이동 결과 알림)

```fbs
table ResultMove {
    id: string;
    goal_id: string;
    goal_name: string;
    method: string;
    preset: int;
    goal_pose: MovePose;
    target: float;
    speed: float;
    direction: string;
    result: string;
    message: string;
}
```

---

## 5. slamnav_multi.fbs - 멀티 로봇

다중 로봇 환경에서의 경로 및 가상 장애물 관리입니다.

### 5.1 경로 공유

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestMultiPath` | 경로 요청 | `id`, `path: [string]` |
| `ResponseMultiPath` | 경로 응답 | `id`, `path: [string]`, `result`, `message` |

### 5.2 VOBS (가상 장애물)

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestMultiVobs` | VOBS 요청 | `id`, `vobs_robots: [string]`, `vobs_closures: [string]`, `is_vobs_c: string` |
| `ResponseMultiVobs` | VOBS 응답 | `id`, `vobs_robots`, `vobs_closures`, `is_vobs_c`, `result`, `message` |

---

## 6. slamnav_setting.fbs - 설정

로봇의 설정 파라미터 관리 스키마입니다.

### 6.1 Robot Type

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetRobotType` | 로봇 타입 조회 | `id` |
| `ResponseGetRobotType` | 로봇 타입 응답 | `id`, `robot_type`, `result`, `message` |
| `RequestSetRobotType` | 로봇 타입 설정 | `id`, `robot_type` |
| `ResponseSetRobotType` | 로봇 타입 설정 응답 | `id`, `robot_type`, `result`, `message` |

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
| `RequestGetPduParam` | PDU 파라미터 조회 | `id` |
| `ResponseGetPduParam` | PDU 파라미터 응답 | `id`, `params: [SettingParam]`, `result`, `message` |
| `RequestSetPduParam` | PDU 파라미터 설정 | `id`, `params: [SettingParam]` |
| `ResponseSetPduParam` | PDU 파라미터 설정 응답 | `id`, `params`, `result`, `message` |

### 6.4 Drive Param

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetDriveParam` | 드라이브 파라미터 조회 | `id` |
| `ResponseGetDriveParam` | 드라이브 파라미터 응답 | `id`, `params: [SettingParam]`, `result`, `message` |

### 6.5 Sensor Info

```fbs
table SensorInfo {
    index: int;
    serial: string;
}
```

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetSensorInfo` | 센서 정보 조회 | `id`, `target: string` |
| `ResponseGetSensorInfo` | 센서 정보 응답 | `id`, `target`, `index: [SensorInfo]`, `result`, `message` |
| `RequestSetSensorInfo` | 센서 정보 설정 | `id`, `target`, `index: [SensorInfo]` |
| `ResponseSetSensorInfo` | 센서 정보 설정 응답 | `id`, `target`, `index`, `result`, `message` |

### 6.6 Sensor Control

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestGetSensorControl` | 센서 제어 상태 조회 | `id`, `target: string`, `control: bool` |
| `ResponseGetSensorControl` | 센서 제어 상태 응답 | `id`, `target`, `control`, `result`, `message` |
| `RequestSetSensorControl` | 센서 제어 설정 | `id`, `target: string`, `control: bool` |
| `ResponseSetSensorControl` | 센서 제어 설정 응답 | `id`, `target`, `control`, `result`, `message` |

### 6.7 Result (결과 알림)

| Table | 설명 | 필드 |
|-------|------|------|
| `ResultSettingParam` | 설정 파라미터 결과 응답 | `id`, `params: [SettingParam]`, `result`, `message` |
| `ResultSettingSensor` | 센서 설정 결과 응답 | `id`, `target`, `control`, `index: [SensorInfo]`, `result`, `message` |

---

## 7. slamnav_socket.fbs - 센서 데이터 스트림

실시간 센서 데이터 스트리밍을 위한 스키마입니다.

### 7.1 Point 구조체

```fbs
struct Point2D {
    x: float;
    y: float;
    intensity: float;
}

struct Point3D {
    x: float;
    y: float;
    z: float;
    intensity: float;
}
```

### 7.2 Lidar 데이터

| Table | 설명 | 필드 |
|-------|------|------|
| `Lidar2D` | 2D 라이다 데이터 (Pub/Sub) | `id: string`, `points: [Point2D]` |
| `RequestLidar3D` | 3D 라이다 요청 (RPC) | `id: string` |
| `ResponseLidar3D` | 3D 라이다 응답 (RPC) | `id: string`, `points: [Point3D]` |
| `MappingCloud` | 맵핑 클라우드 데이터 (Pub/Sub) | `id: string`, `points: [Point3D]` |

### 7.3 Path 데이터 (주석처리됨)

> **참고:** GlobalPath, LocalPath는 현재 주석처리되어 있으며, multi_path와 통합 예정입니다.

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
    localization: string;      // "good" / "none" / "fail"
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

### 8.8 종합 Status (Pub/Sub)

```fbs
table Status {
    condition: StatusCondition;
    imu: StatusImu;
    motor: [StatusMotor];      // 모터 배열
    power: StatusPower;
    robot_state: StatusRobotState;
    robot_safety_io_state: StatusRobotSafetyIoState;
    setting: StatusSetting;
    map: StatusMap;
}
```

### 8.9 Move Status (이동 상태)

```fbs
table StatusMoveState {
    auto_move: string;     // "stop" / "move" / "pause" / "error" / "not ready" / "vir"
    dock_move: string;     // "stop"
    jog_move: string;      // "none"
    obs: string;           // "none" / "near" / "far" / "vir"
    path_state: string;    // "none" / "req_path" / "recv_path"
    multi_id: string;      // multi 경로 id
    step: int;             // 로봇 경로 step
    multi_state: string;   // "none" / "move" / "complete" / "fail" / "obstacle" / "cancel"
}

struct StatusPose {
    x: float;
    y: float;
    rz: float;
}

struct StatusVel {
    vx: float;
    vy: float;
    wz: float;
}

table StatusNode {
    id: string;
    name: string;
    pose: StatusPose;
}

table MoveStatus {
    move_state: StatusMoveState;
    pose: StatusPose;
    vel: StatusVel;
    cur_node: StatusNode;
    goal_node: StatusNode;
}
```

---

## 9. slamnav_update.fbs - 업데이트

소프트웨어 업데이트 관련 스키마입니다.

| Table | 설명 | 필드 |
|-------|------|------|
| `RequestUpdate` | 업데이트 요청 | `id`, `branch`, `version` |
| `ResponseUpdate` | 업데이트 응답 | `id`, `branch`, `version`, `result`, `message` |
| `RequestCurrentVersion` | 현재 버전 조회 | `id` |
| `ResponseCurrentVersion` | 현재 버전 응답 | `id`, `version`, `result`, `message` |
| `ResultUpdate` | 업데이트 결과 알림 | `id`, `branch`, `version`, `result`, `message` |

---

## 공통 패턴

### Request/Response 패턴

모든 명령은 Request/Response 패턴을 따릅니다:
- **Request**: `id` 필드 포함 (요청 식별자)
- **Response**: `id`, `result`, `message` 필드 포함
  - `result`: `"success"` 또는 `"fail"`
  - `message`: 실패 시 사유

### Result 테이블

비동기 작업의 결과를 알리는 테이블:
- `ResultControlDock`
- `ResultLocalizationInit`
- `ResultMapLoad`
- `ResultMove`
- `ResultSettingParam`
- `ResultSettingSensor`
- `ResultUpdate`

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
