# SLAM Zenoh Topics

SLAM 서비스와 AMR 간의 Zenoh 통신 토픽 정의.

---

## 스키마 파일 구조

```
schemas/slam/
├── slamnav_move.fbs          # 이동 관련 RPC + Pub/Sub (Jog)
├── slamnav_localization.fbs  # 위치추정 관련 RPC
├── slamnav_control.fbs       # 제어 관련 RPC (Safety, Dock, LED, Motor, Sensor 등)
├── slamnav_map.fbs           # 맵 관련 RPC + Mapping
├── slamnav_setting.fbs       # 설정 관련 RPC (Robot Type, Sensor, PDU)
├── slamnav_update.fbs        # 소프트웨어 업데이트 RPC
├── slamnav_status.fbs        # 로봇 상태 Pub/Sub (Status, Move_Status)
├── slamnav_socket.fbs        # 센서 데이터 Pub/Sub (Lidar_2D, Lidar_3D, Mapping_Cloud, Path)
├── slamnav_multi.fbs         # 멀티로봇 관련 RPC (Path, Vobs)
└── topics.md                 # 토픽 문서 (본 파일)
```

---

## 로봇 모델 (Robot Models)

토픽의 `{model}` 부분에 사용되는 로봇 모델 식별자 정의.
whoami 로 호출한 serial_num을 사용.

### 모델별 토픽 예시

{robot_serial}/move/goal          # 로봇 목표점 이동

---


## 통신 init 방법

SLAMNAN가 Whoami로 RRS에 연결 시도.
응답으로 {robot_serial} 을 제공받고, 모든 topic 최상단에 정의.
    - {robot_serial} 이 없을 경우, Robot_Model로 반환.
    - {robot_serial} 변경 시, 모든 topic을 중지하고, 변경받은 {robot_serial}로 재발행.
    - {robot_serial} 제공 전, 어떠한 topic도 발행하지 않음.


## Topic 구조

```
{model}/                    # 로봇 모델명 (S100, D400 등)
├── move/                   # 이동 관련 (slamnav_move.fbs)
├── localization/           # 위치추정 관련 (slamnav_localization.fbs)
├── control/                # 제어 관련 (slamnav_control.fbs)
├── map/                    # 맵 관련 (slamnav_map.fbs)
├── setting/                # 설정 관련 (slamnav_setting.fbs)
├── software/               # 소프트웨어 관련 (slamnav_update.fbs)
├── multi/                  # 멀티로봇 관련 (slamnav_multi.fbs)
├── status                  # Pub/Sub - 로봇 상태 (slamnav_status.fbs)
├── moveStatus              # Pub/Sub - 이동 상태 (slamnav_status.fbs)
├── lidar2d                 # Pub/Sub - 2D 라이다 (slamnav_socket.fbs)
├── lidar3d                 # Pub/Sub - 3D 라이다 (slamnav_socket.fbs)
├── mappingCloud            # Pub/Sub - 매핑 클라우드 (slamnav_socket.fbs)
├── globalPath              # Pub/Sub - 전역 경로 (slamnav_socket.fbs)
└── localPath               # Pub/Sub - 지역 경로 (slamnav_socket.fbs)
```

---

## RPC (Queryable)

Zenoh Queryable을 사용한 Request/Response 패턴.

### 이동 (Move) - `{model}/move/*`

> 스키마 파일: `slamnav_move.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/move/goal` | `Request_Move_Goal` | `Response_Move_Goal` | 목표점 이동 (goal_id, goal_name, method, preset) |
| `{model}/move/target` | `Request_Move_Target` | `Response_Move_Target` | 목표 포즈 이동 (goal_pose: MovePose, method, preset) |
| `{model}/move/stop` | `Request_Move_Stop` | `Response_Move_Stop` | 정지 |
| `{model}/move/pause` | `Request_Move_Pause` | `Response_Move_Pause` | 일시정지 |
| `{model}/move/resume` | `Request_Move_Resume` | `Response_Move_Resume` | 재개 |
| `{model}/move/xLinear` | `Request_Move_XLinear` | `Response_Move_XLinear` | X축 이동 (target, speed) |
| `{model}/move/circular` | `Request_Move_Circular` | `Response_Move_Circular` | Circular 이동 (target, speed, direction) |
| `{model}/move/rotate` | `Request_Move_Rotate` | `Response_Move_Rotate` | 회전 이동 (target, speed) |

### 조그 (Jog) - Pub/Sub

> 스키마 파일: `slamnav_move.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/move/jog` | `Move_Jog` | 조그 이동 (vx, vy, wz) |

### 이동 상태 변경 이벤트

> 스키마 파일: `slamnav_move.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/move/stateChange` | `State_Change_Move` | 이동 상태 변경 이벤트 |

### 이동 결과 응답

> 스키마 파일: `slamnav_move.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/move/result` | `Move_Result` | 이동 결과 (id, result, message) |

### 위치추정 (Localization) - `{model}/localization/*`

> 스키마 파일: `slamnav_localization.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/localization/init` | `Request_Localization_Init` | `Response_Localization_Init` | 수동 초기 위치 설정 (pose: LocalizationPose) |
| `{model}/localization/autoinit` | `Request_Localization_AutoInit` | `Response_Localization_AutoInit` | 자동 초기화 |
| `{model}/localization/semiautoinit` | `Request_Localization_SemiAutoInit` | `Response_Localization_SemiAutoInit` | 반자동 초기화 |
| `{model}/localization/randominit` | `Request_Localization_RandomInit` | `Response_Localization_RandomInit` | 랜덤 초기화 (random_seed) |
| `{model}/localization/start` | `Request_Localization_Start` | `Response_Localization_Start` | 위치추정 시작 |
| `{model}/localization/stop` | `Request_Localization_Stop` | `Response_Localization_Stop` | 위치추정 정지 |

### 위치추정 결과 응답

> 스키마 파일: `slamnav_localization.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/localization/result` | `Localization_Result` | 위치추정 결과 (id, result, message) |

### 제어 (Control) - `{model}/control/*`

> 스키마 파일: `slamnav_control.fbs`

#### Safety 관련

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/getSafetyField` | `Request_Get_Safety_Field` | `Response_Get_Safety_Field` | 안전 필드 조회 |
| `{model}/control/setSafetyField` | `Request_Set_Safety_Field` | `Response_Set_Safety_Field` | 안전 필드 설정 |
| `{model}/control/getSafetyFlag` | `Request_Get_Safety_Flag` | `Response_Get_Safety_Flag` | 안전 플래그 조회 |
| `{model}/control/setSafetyFlag` | `Request_Set_Safety_Flag` | `Response_Set_Safety_Flag` | 안전 플래그 리셋/설정 |
| `{model}/control/getSafetyIo` | `Request_Get_Safety_Io` | `Response_Get_Safety_Io` | Safety I/O 조회 |
| `{model}/control/setSafetyIo` | `Request_Set_Safety_Io` | `Response_Set_Safety_Io` | Safety I/O 설정 |

#### Dock 관련

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/dock` | `Request_Dock` | `Response_Dock` | 도킹/언도킹/정지 (command: dock/undock/dockstop) |
| `{model}/control/chargeTrigger` | `Request_Charge_Trigger` | `Response_Charge_Trigger` | 충전 트리거 On/Off |

#### Dock 상태 변경 이벤트

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/control/dock/stateChange` | `State_Change_Dock` | 도킹 상태 변경 이벤트 |

#### 장애물 박스

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/getObsBox` | `Request_Get_Obs_Box` | `Response_Get_Obs_Box` | 장애물 박스 조회 (min, max: ObsBox, range) |
| `{model}/control/setObsBox` | `Request_Set_Obs_Box` | `Response_Set_Obs_Box` | 장애물 박스 설정 (min, max: ObsBox, range) |

#### LED/Motor/Jog

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/led` | `Request_Led` | `Response_Led` | LED 제어 (onoff, color) |
| `{model}/control/motor` | `Request_Motor` | `Response_Motor` | 모터 On/Off |
| `{model}/control/jog` | `Request_Jog` | `Response_Jog` | 조그 모드 On/Off |

#### 센서/경로 주파수 설정

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/sensor` | `Request_Sensor` | `Response_Sensor` | 센서 소켓 설정 (command: camera/lidar2d/lidar3d, onoff, frequency) |
| `{model}/control/path` | `Request_Path` | `Response_Path` | 경로 Pub 설정 (onoff, frequency) |

#### 카메라 검출

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/detect` | `Request_Detect` | `Response_Detect` | 카메라 검출 (camera_number, camera_serial, size, tf 반환) |

### 맵 (Map) - `{model}/map/*`

> 스키마 파일: `slamnav_map.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/map/getList` | `Request_Map_List` | `Response_Map_List` | 맵 목록 조회 (list: [MapFile]) |
| `{model}/map/getCurrent` | `Request_Map_Current` | `Response_Map_Current` | 현재 로드된 맵 조회 |
| `{model}/map/load` | `Request_Map_Load` | `Response_Map_Load` | 맵 로드 |
| `{model}/map/delete` | `Request_Map_Delete` | `Response_Map_Delete` | 맵 삭제 |
| `{model}/map/getFile` | `Request_Get_Map_File` | `Response_Get_Map_File` | 맵 파일 조회 (바이너리 데이터) |
| `{model}/map/getCloud` | `Request_Get_Map_Cloud` | `Response_Get_Map_Cloud` | 맵 포인트클라우드 조회 |
| `{model}/map/setCloud` | `Request_Set_Map_Cloud` | `Response_Set_Map_Cloud` | 맵 포인트클라우드 설정 |
| `{model}/map/getTopology` | `Request_Get_Map_Topology` | `Response_Get_Map_Topology` | 토폴로지 조회 |
| `{model}/map/setTopology` | `Request_Set_Map_Topology` | `Response_Set_Map_Topology` | 토폴로지 설정 |
| `{model}/map/mapping/start` | `Request_Mapping_Start` | `Response_Mapping_Start` | 매핑 시작 |
| `{model}/map/mapping/stop` | `Request_Mapping_Stop` | `Response_Mapping_Stop` | 매핑 정지 |
| `{model}/map/mapping/save` | `Request_Mapping_Save` | `Response_Mapping_Save` | 맵 저장 |

### 맵 결과 응답

> 스키마 파일: `slamnav_map.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/map/result` | `Map_Result` | 맵 결과 (id, result, message) |

### 설정 (Setting) - `{model}/setting/*`

> 스키마 파일: `slamnav_setting.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/setting/getRobotType` | `Request_Get_Robot_Type` | `Response_Get_Robot_Type` | 로봇 타입 조회 |
| `{model}/setting/setRobotType` | `Request_Set_Robot_Type` | `Response_Set_Robot_Type` | 로봇 타입 설정 |
| `{model}/setting/getSensorIndex` | `Request_Get_Sensor_Index` | `Response_Get_Sensor_Index` | 센서 인덱스 조회 (target) |
| `{model}/setting/setSensorIndex` | `Request_Set_Sensor_Index` | `Response_Set_Sensor_Index` | 센서 인덱스 설정 (target, index) |
| `{model}/setting/setSensorOn` | `Request_Set_Sensor_On` | `Response_Set_Sensor_On` | 센서 켜기 |
| `{model}/setting/getSensorOff` | `Request_Get_Sensor_Off` | `Response_Get_Sensor_Off` | 센서 끄기 |
| `{model}/setting/getPduParam` | `Request_Get_Pdu_Param` | `Response_Get_Pdu_Param` | PDU 파라미터 조회 |
| `{model}/setting/setPduParam` | `Request_Set_Pdu_Param` | `Response_Set_Pdu_Param` | PDU 파라미터 설정 |
| `{model}/setting/getDriveParam` | `Request_Get_Drive_Param` | `Response_Get_Drive_Param` | 드라이브 파라미터 조회 |

### 설정 결과 응답

> 스키마 파일: `slamnav_setting.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/setting/result` | `Setting_Result` | 설정 결과 (id, result, message) |

### 소프트웨어 (Software) - `{model}/software/*`

> 스키마 파일: `slamnav_update.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/software/update` | `Request_Update` | `Response_Update` | 소프트웨어 업데이트 (branch, version) |
| `{model}/software/getVersion` | `Request_Current_Version` | `Response_Current_Version` | 현재 버전 조회 |

### 업데이트 결과 응답

> 스키마 파일: `slamnav_update.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/software/result` | `Update_Result` | 업데이트 결과 (id, result, message) |

### 멀티로봇 (Multi) - `{model}/multi/*`

> 스키마 파일: `slamnav_multi.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/multi/path` | `Request_Path` | `Response_Path` | 경로 설정/조회 (path: [string]) |
| `{model}/multi/vobs` | `Request_Vobs` | `Response_Vobs` | 가상 장애물 설정 (vobs_robots, vobs_closures: [string]) |

---

## Pub/Sub (주기적/실시간 데이터)

### 상태 (Status)

> 스키마 파일: `slamnav_status.fbs`

| Topic | Message Type | Period | Description |
|-------|--------------|--------|-------------|
| `{model}/status` | `Status` | 100ms | 로봇 상태 (IMU, Motor, Condition, Robot State, Safety I/O, Power, Setting, Map) |
| `{model}/moveStatus` | `Move_Status` | 500ms | 이동 상태 (Move State, Pose, Vel, Goal Node, Cur Node) |

### 상태 결과 응답

> 스키마 파일: `slamnav_status.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/status/result` | `Status_Result` | 상태 결과 (id, result, message) |

#### Status 구조

```
Status (table)
├── condition: StatusCondition (struct)     # 위치추정 상태 (inlier_error/ratio, mapping_error/ratio)
├── imu: StatusImu (struct)                 # IMU 데이터 (acc, gyr, rotation)
├── motor0: StatusMotor (struct)            # 모터0 상태 (connection, status, temp, current)
├── motor1: StatusMotor (struct)            # 모터1 상태
├── power: StatusPower (struct)             # 전원 상태 (배터리, TABOS 등)
├── robot_state: StatusRobotState (table)   # 로봇 상태 (charge, dock, emo, localization, power)
├── robot_safety_io_state: StatusRobotSafetyIoState (table)  # Safety I/O 상태
├── setting: StatusSetting (table)          # 설정 (platform_type, platform_name)
└── map: StatusMap (table)                  # 맵 상태 (map_name, map_status)
```

#### Move_Status 구조

```
Move_Status (table)
├── cur_node: MoveStatusNode (table)        # 현재 노드 정보
├── goal_node: MoveStatusNode (table)       # 목표 노드 정보
├── move_state: MoveStatusMoveState (table) # 이동 상태 (auto_move, dock_move, jog_move, obs, path)
├── pose: MoveStatusPose (struct)           # 현재 위치 (x, y, z, rz)
└── vel: MoveStatusVel (struct)             # 현재 속도 (vx, vy, wz)
```

### 센서 데이터 (Sensor Data)

> 스키마 파일: `slamnav_socket.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/lidar2d` | `Lidar_2D` | 2D 라이다 데이터 (timestamp, points: [Point2D]) |
| `{model}/lidar3d` | `Lidar_3D` | 3D 라이다 데이터 (timestamp, points: [Point3D]) |
| `{model}/mappingCloud` | `Mapping_Cloud` | 매핑 포인트클라우드 (timestamp, points: [Point3D]) |

### 경로 데이터 (Path Data)

> 스키마 파일: `slamnav_socket.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/globalPath` | `Global_Path` | 전역 경로 (id, path: [string]) |
| `{model}/localPath` | `Local_Path` | 지역 경로 (id, path: [string]) |

---

## Zenoh 사용 예시

```cpp
// RPC 호출 (Client)
auto reply = session.get("{model}/move/goal", payload);

// RPC 응답 (Queryable)
session.declare_queryable("{model}/move/goal", handler);

// Jog 명령 발행 (Publisher)
session.put("{model}/move/jog", jog_payload);

// Jog 명령 수신 (Subscriber)
session.declare_subscriber("{model}/move/jog", callback);

// 상태 수신 (Subscriber)
session.declare_subscriber("{model}/status", callback);
session.declare_subscriber("{model}/moveStatus", callback);

// 경로 수신
session.declare_subscriber("{model}/globalPath", callback);
session.declare_subscriber("{model}/localPath", callback);

// 특정 로봇 전체 데이터 수신 (와일드카드/모니터링)
session.declare_subscriber("{model}/**", callback);
```

---

## QoS 설정 가이드

| Topic Type | Pattern | Reliability | Priority |
|------------|---------|-------------|----------|
| RPC | `{model}/move/*` 등 | Reliable | High |
| Jog | `{model}/move/jog` | BestEffort | High |
| Status | `{model}/status`, `{model}/moveStatus` | BestEffort | Medium |
| Path | `{model}/globalPath`, `{model}/localPath` | BestEffort | Medium |

---

## 공통 타입

### Node (slamnav_map.fbs) - table

| 필드 | 타입 | 설명 |
|------|------|------|
| `id` | string | 노드 ID |
| `name` | string | 노드 이름 |
| `pose` | [float] | 노드 위치 |
| `info` | string | 노드 정보 |
| `links` | [Link] | 연결된 링크 목록 |
| `type` | string | 노드 타입 |

### Link (slamnav_map.fbs) - table

| 필드 | 타입 | 설명 |
|------|------|------|
| `id` | string | 링크 ID (연결된 노드 ID) |
| `info` | string | 링크 정보 |
| `speed` | float | 링크 속도 |
| `method` | string | 이동 방식 |
| `safety_field` | int | 안전 필드 설정 |

### MapFileInfo (slamnav_map.fbs) - table

| 필드 | 타입 | 설명 |
|------|------|------|
| `map_name` | string | 맵 이름 |
| `created_at` | string | 생성 시간 |
| `update_at` | string | 수정 시간 |
| `type` | string | 파일 타입 |
| `size` | float | 파일 크기 |

### MapFile (slamnav_map.fbs) - table

| 필드 | 타입 | 설명 |
|------|------|------|
| `file_name` | string | 파일 이름 |
| `created_at` | string | 생성 시간 |
| `update_at` | string | 수정 시간 |
| `type` | string | 파일 타입 |
| `cloud_info` | MapFileInfo | 클라우드 정보 |
| `topo_info` | MapFileInfo | 토폴로지 정보 |

### CloudData (slamnav_map.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | float | X 좌표 |
| `y` | float | Y 좌표 |
| `z` | float | Z 좌표 |
| `intensity` | float | 강도 |

### SettingParam (slamnav_setting.fbs) - table

| 필드 | 타입 | 설명 |
|------|------|------|
| `key` | string | 파라미터 키 |
| `type` | string | 파라미터 타입 |
| `value` | string | 파라미터 값 |

### SensorInfo (slamnav_setting.fbs) - table

| 필드 | 타입 | 설명 |
|------|------|------|
| `id` | int | 센서 ID |
| `serial` | string | 센서 시리얼 번호 |

### SafetyFlag (slamnav_control.fbs) - table

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | string | 플래그 이름 |
| `value` | bool | 플래그 값 |

### ObsBox (slamnav_control.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | float | X 좌표 |
| `y` | float | Y 좌표 |
| `z` | float | Z 좌표 |

### Point2D (slamnav_socket.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | float | X 좌표 |
| `y` | float | Y 좌표 |

### Point3D (slamnav_socket.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | float | X 좌표 |
| `y` | float | Y 좌표 |
| `z` | float | Z 좌표 |

### LocalizationPose (slamnav_localization.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | float | X 좌표 |
| `y` | float | Y 좌표 |
| `z` | float | Z 좌표 |
| `rz` | float | Z축 회전 |

### MovePose (slamnav_move.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | float | X 좌표 |
| `y` | float | Y 좌표 |
| `z` | float | Z 좌표 |
| `rz` | float | Z축 회전 |

### MoveStatusPose (slamnav_status.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | float | X 좌표 |
| `y` | float | Y 좌표 |
| `z` | float | Z 좌표 |
| `rz` | float | Z축 회전 |

### MoveStatusVel (slamnav_status.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `vx` | float | X축 속도 |
| `vy` | float | Y축 속도 |
| `wz` | float | Z축 각속도 |

### StatusImu (slamnav_status.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `acc_x` | float | X축 가속도 |
| `acc_y` | float | Y축 가속도 |
| `acc_z` | float | Z축 가속도 |
| `gyr_x` | float | X축 각속도 |
| `gyr_y` | float | Y축 각속도 |
| `gyr_z` | float | Z축 각속도 |
| `imu_rx` | float | X축 회전 |
| `imu_ry` | float | Y축 회전 |
| `imu_rz` | float | Z축 회전 |

### StatusMotor (slamnav_status.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `connection` | bool | 연결 상태 |
| `status` | int | 모터 상태 |
| `temp` | float | 온도 |
| `current` | float | 전류 |

### StatusCondition (slamnav_status.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `inlier_error` | float | 인라이어 에러 |
| `inlier_ratio` | float | 인라이어 비율 |
| `mapping_error` | float | 매핑 에러 |
| `mapping_ratio` | float | 매핑 비율 |

### StatusPower (slamnav_status.fbs) - struct

| 필드 | 타입 | 설명 |
|------|------|------|
| `bat_in` | float | 배터리 입력 전압 |
| `bat_out` | float | 배터리 출력 전압 |
| `bat_current` | float | 배터리 전류 |
| `total_power` | float | 총 전력 |
| `power` | float | 전력 |
| `bat_percent` | float | 배터리 잔량 (%) |
| `tabos_voltage` | float | TABOS 전압 |
| `tabos_current` | float | TABOS 전류 |
| `tabos_status` | float | TABOS 상태 |
| `tabos_ttf` | float | TABOS TTF |
| `tabos_tte` | float | TABOS TTE |
| `tabos_soc` | float | TABOS SOC |
| `tabos_soh` | float | TABOS SOH |
| `tabos_temp` | float | TABOS 온도 |
| `tabos_rc` | float | TABOS RC |
| `tabos_ae` | float | TABOS AE |
| `charge_current` | float | 충전 전류 |
| `contact_voltage` | float | 접점 전압 |
