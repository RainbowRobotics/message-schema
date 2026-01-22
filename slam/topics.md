# SLAM Zenoh Topics

SLAM 서비스와 AMR 간의 Zenoh 통신 토픽 정의.

---

## 스키마 파일 구조

```
schemas/slam/
├── slamnav_move.fbs          # 이동 관련 RPC + Pub/Sub (Jog)
├── slamnav_localization.fbs  # 위치추정 관련 RPC
├── slamnav_control.fbs       # 제어 관련 RPC (Safety, Dock, LED, Motor 등)
├── slamnav_map.fbs           # 맵 관련 RPC + Mapping
├── slamnav_path.fbs          # 경로 Pub/Sub
├── slamnav_setting.fbs       # 설정 관련 RPC (Robot Type, Sensor, PDU)
├── slamnav_update.fbs        # 소프트웨어 업데이트 RPC
├── slamnav_status.fbs        # 로봇 상태 Pub/Sub
├── slamnav_moveStatus.fbs    # 이동 상태 Pub/Sub
└── topics.md                 # 토픽 문서 (본 파일)
```

---

## 로봇 모델 (Robot Models)

토픽의 `{model}` 부분에 사용되는 로봇 모델 식별자 정의.

| Model | Description |
|-------|-------------|
| `S100` | 소형 로봇 |
| `D400` | 중형 로봇 |
| `D1000` | 대형 로봇 |

### 모델별 토픽 예시

```
S100/move/goal          # S100 로봇 목표점 이동
D400/status             # D400 로봇 상태
D1000/localization/init # D1000 로봇 위치 초기화
```

---

## Topic 구조

```
{model}/                    # 로봇 모델명 (S100, D400 등)
├── move/                   # 이동 관련 (slamnav_move.fbs)
├── localization/           # 위치추정 관련 (slamnav_localization.fbs)
├── control/                # 제어 관련 (slamnav_control.fbs)
├── map/                    # 맵 관련 (slamnav_map.fbs)
├── setting/                # 설정 관련 (slamnav_setting.fbs)
├── software/               # 소프트웨어 관련 (slamnav_update.fbs)
├── status                  # Pub/Sub - 로봇 상태 (slamnav_status.fbs)
├── moveStatus              # Pub/Sub - 이동 상태 (slamnav_moveStatus.fbs)
├── globalPath              # Pub/Sub - 전역 경로 (slamnav_path.fbs)
└── localPath               # Pub/Sub - 지역 경로 (slamnav_path.fbs)
```

---

## RPC (Queryable)

Zenoh Queryable을 사용한 Request/Response 패턴.

### 이동 (Move) - `{model}/move/*`

> 스키마 파일: `slamnav_move.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/move/goal` | `Request_Move_Goal` | `Response_Move_Goal` | 목표점 이동 (goal_id, method, preset) |
| `{model}/move/target` | `Request_Move_Target` | `Response_Move_Target` | 목표 포즈 이동 (goal_pose, method, preset) |
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
| `{model}/move/jog` | `Request_Move_Jog` | 조그 이동 (vx, vy, wz) |

### 이동 상태 변경 이벤트

> 스키마 파일: `slamnav_move.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/move/stateChange` | `State_Change_Move` | 이동 상태 변경 이벤트 |

### 위치추정 (Localization) - `{model}/localization/*`

> 스키마 파일: `slamnav_localization.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/localization/init` | `Request_Localization_Init` | `Response_Localization_Init` | 수동 초기 위치 설정 (x, y, z, rz) |
| `{model}/localization/autoinit` | `Request_Localization_AutoInit` | `Response_Localization_AutoInit` | 자동 초기화 |
| `{model}/localization/semiautoinit` | `Request_Localization_SemiAutoInit` | `Response_Localization_SemiAutoInit` | 반자동 초기화 |
| `{model}/localization/randominit` | `Request_Localization_RandomInit` | `Response_Localization_RandomInit` | 랜덤 초기화 (random_seed) |
| `{model}/localization/start` | `Request_Localization_Start` | `Response_Localization_Start` | 위치추정 시작 |
| `{model}/localization/stop` | `Request_Localization_Stop` | `Response_Localization_Stop` | 위치추정 정지 |

### 제어 (Control) - `{model}/control/*`

> 스키마 파일: `slamnav_control.fbs`

#### Safety 관련

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/safetyField` | `Request_Safety_Field` | `Response_Safety_Field` | 안전 필드 설정 (command: get/set) |
| `{model}/control/resetSafetyFlag` | `Request_Reset_Safety_Flag` | `Response_Reset_Safety_Flag` | 안전 플래그 리셋 |
| `{model}/control/safetyIo` | `Request_Safety_Io` | `Response_Safety_Io` | Safety I/O 조회/설정 (command: get/set) |

#### Dock 관련

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/dock` | `Request_Dock` | `Response_Dock` | 도킹 |
| `{model}/control/undock` | `Request_Undock` | `Response_Undock` | 언도킹 |
| `{model}/control/dockStop` | `Request_DockStop` | `Response_DockStop` | 도킹 정지 |

#### 장애물 박스

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/obsBox` | `Request_Obs_Box` | `Response_Obs_Box` | 장애물 박스 설정 (command: get/set, min_z, max_z, map_range) |

#### LED/Motor

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/led` | `Request_Led` | `Response_Led` | LED 제어 (onoff, color) |
| `{model}/control/motor` | `Request_Motor` | `Response_Motor` | 모터 On/Off |

#### 시퀀스/주파수 설정

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/randomSequence` | `Request_Random_Sequence` | `Response_Random_Sequence` | 랜덤 시퀀스 실행 |
| `{model}/control/frequency` | `Request_Frequency` | `Response_Frequency` | Pub/Sub 주파수 설정 (target: lidar/path, onoff, frequency) |

#### Dock 상태 변경 이벤트

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/control/dock/stateChange` | `State_Change_Dock` | 도킹 상태 변경 이벤트 |

### 맵 (Map) - `{model}/map/*`

> 스키마 파일: `slamnav_map.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/map/load` | `Request_Map_Load` | `Response_Map_Load` | 맵 로드 |
| `{model}/map/getList` | `Request_Map_List` | `Response_Map_List` | 맵 목록 조회 |
| `{model}/map/getFile` | `Request_Get_Map_File` | `Response_Get_Map_File` | 맵 파일 조회 (바이너리 데이터) |
| `{model}/map/getCloud` | `Request_Get_Map_Cloud` | `Response_Get_Map_Cloud` | 맵 포인트클라우드 조회 |
| `{model}/map/setCloud` | `Request_Set_Map_Cloud` | `Response_Set_Map_Cloud` | 맵 포인트클라우드 설정 |
| `{model}/map/getTopology` | `Request_Get_Map_Topology` | `Response_Get_Map_Topology` | 토폴로지 조회 (페이지네이션 지원) |
| `{model}/map/setTopology` | `Request_Set_Map_Topology` | `Response_Set_Map_Topology` | 토폴로지 설정 |
| `{model}/map/mapping/start` | `Request_Mapping_Start` | `Response_Mapping_Start` | 매핑 시작 |
| `{model}/map/mapping/stop` | `Request_Mapping_Stop` | `Response_Mapping_Stop` | 매핑 정지 |
| `{model}/map/mapping/save` | `Request_Mapping_Save` | `Response_Mapping_Save` | 맵 저장 |

### 설정 (Setting) - `{model}/setting/*`

> 스키마 파일: `slamnav_setting.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/setting/getRobotType` | `Request_Get_Robot_Type` | `Response_Get_Robot_Type` | 로봇 타입 조회 |
| `{model}/setting/setRobotType` | `Request_Set_Robot_Type` | `Response_Set_Robot_Type` | 로봇 타입 설정 |
| `{model}/setting/getSensorIndex` | `Request_Get_Sensor_Index` | `Response_Get_Sensor_Index` | 센서 인덱스 조회 |
| `{model}/setting/setSensorIndex` | `Request_Set_Sensor_Index` | `Response_Set_Sensor_Index` | 센서 인덱스 설정 |
| `{model}/setting/setSensorOn` | `Request_Set_Sensor_On` | `Response_Set_Sensor_On` | 센서 켜기 |
| `{model}/setting/getSensorOff` | `Request_Get_Sensor_Off` | `Response_Get_Sensor_Off` | 센서 끄기 |
| `{model}/setting/getPduParam` | `Request_Get_Pdu_Param` | `Response_Get_Pdu_Param` | PDU 파라미터 조회 |
| `{model}/setting/setPduParam` | `Request_Set_Pdu_Param` | `Response_Set_Pdu_Param` | PDU 파라미터 설정 |

### 소프트웨어 (Software) - `{model}/software/*`

> 스키마 파일: `slamnav_update.fbs`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/software/update` | `Request_Update` | `Response_Update` | 소프트웨어 업데이트 (branch, version) |
| `{model}/software/getVersion` | `Request_Current_Version` | `Response_Current_Version` | 현재 버전 조회 |

---

## Pub/Sub (주기적/실시간 데이터)

### 상태 (Status)

> 스키마 파일: `slamnav_status.fbs`, `slamnav_moveStatus.fbs`

| Topic | Message Type | Period | Description |
|-------|--------------|--------|-------------|
| `{model}/status` | `Status` | 100ms | 로봇 상태 (IMU, Motor, Condition, Robot State, Safety I/O, Power, Setting, Map) |
| `{model}/moveStatus` | `MoveStatus` | 500ms | 이동 상태 (Move State, Pose, Vel, Goal Node, Cur Node) |

#### Status 구조

```
Status
├── imu: Status_Imu                         # IMU 데이터 (acc, gyr, rotation)
├── motor0: Status_Motor                    # 모터0 상태 (connection, status, temp, current)
├── motor1: Status_Motor                    # 모터1 상태
├── condition: Status_Condition             # 위치추정 상태 (inlier_error/ratio, mapping_error/ratio)
├── robot_state: Status_Robot_State         # 로봇 상태 (charge, dock, emo, localization, power)
├── robot_safety_io_state: Status_Robot_Safety_Io_State  # Safety I/O 상태
├── power: Status_Power                     # 전원 상태 (배터리, TABOS 등)
├── setting: Status_Setting                 # 설정 (platform_type, platform_name)
└── map: Status_Map                         # 맵 상태 (map_name, map_status)
```

#### MoveStatus 구조

```
MoveStatus
├── move_state: MoveStatus_Move_State       # 이동 상태 (auto_move, dock_move, jog_move, obs, path)
├── pose: MoveStatus_Pose                   # 현재 위치 (x, y, rz)
├── vel: MoveStatus_Vel                     # 현재 속도 (vx, vy, wz)
├── goal_node: MoveStatus_Node              # 목표 노드 정보
└── cur_node: MoveStatus_Node               # 현재 노드 정보
```

### 경로 (Path)

> 스키마 파일: `slamnav_path.fbs`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/globalPath` | `Path` | 전역 경로 (PointArray 배열) |
| `{model}/localPath` | `Path` | 지역 경로 (PointArray 배열) |

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

### Node (slamnav_map.fbs)

| 필드 | 타입 | 설명 |
|------|------|------|
| `id` | string | 노드 ID |
| `name` | string | 노드 이름 |
| `pose` | [double] | 노드 위치 |
| `info` | string | 노드 정보 |
| `links` | [string] | 연결된 노드 ID 목록 |
| `type` | string | 노드 타입 |

### Setting_Param (slamnav_setting.fbs)

| 필드 | 타입 | 설명 |
|------|------|------|
| `key` | string | 파라미터 키 |
| `type` | string | 파라미터 타입 |
| `value` | string | 파라미터 값 |

### Sensor_Info (slamnav_setting.fbs)

| 필드 | 타입 | 설명 |
|------|------|------|
| `id` | int | 센서 ID |
| `serial` | string | 센서 시리얼 번호 |

### Safety_Flag (slamnav_control.fbs)

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | string | 플래그 이름 |
| `value` | bool | 플래그 값 |
