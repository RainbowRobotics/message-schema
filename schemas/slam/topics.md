# SLAM Zenoh Topics

SLAM 서비스와 AMR 간의 Zenoh 통신 토픽 정의.

---

## 로봇 모델 (Robot Models)

토픽의 `{model}` 부분에 사용되는 로봇 모델 식별자 정의.

| Model | Description | Payload (kg) | Use Case |
|-------|-------------|--------------|----------|
| `S100`
| `D400`
| `D1000`

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
├── move/                   # 이동 관련
├── localization/           # 위치추정 관련
├── control/                # 제어 관련
├── map/                    # 맵 관련
├── path/                   # 경로 관련
├── vobs/                   # 가상 장애물 관련
├── software/               # 소프트웨어 관련
├── config/                 # 설정 관련
├── sensor/                 # 센서 관련
├── status                  # Pub/Sub - 로봇 상태
├── moveStatus              # Pub/Sub - 이동 상태
├── result                  # Pub/Sub - 결과 응답
├── globalPath              # Pub/Sub - 전역 경로
└── localPath               # Pub/Sub - 지역 경로
```

---

## RPC (Queryable)

Zenoh Queryable을 사용한 Request/Response 패턴.

### 이동 (Move) - `{model}/move/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/move/goal` | `MoveGoalRequest` | `MoveResponse` | 목표점 이동 |
| `{model}/move/target` | `MoveTargetRequest` | `MoveResponse` | 목표 거리/각도 이동 |
| `{model}/move/stop` | `MoveStopRequest` | `MoveResponse` | 정지 |
| `{model}/move/pause` | `MovePauseRequest` | `MoveResponse` | 일시정지 |
| `{model}/move/resume` | `MoveResumeRequest` | `MoveResponse` | 재개 |
| `{model}/move/xLinear` | `MoveXLinearRequest` | `MoveResponse` | X축 이동 |
| `{model}/move/yLinear` | `MoveYLinearRequest` | `MoveResponse` | Y축 이동 |
| `{model}/move/circular` | `MoveCircularRequest` | `MoveResponse` | Circular 이동 |
| `{model}/move/rotate` | `MoveRotationRequest` | `MoveResponse` | 회전 이동 |

### 조그 (Jog) | (pub / Sub)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/move/jog` | `MoveJogCommand` | 조그 이동 (Subscriber) |

### 위치추정 (Localization) - `{model}/localization/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/localization/init` | `LocalizationInitRequest` | `LocalizationResponse` | 수동 초기 위치 설정 |
| `{model}/localization/autoinit` | `LocalizationAutoInitRequest` | `LocalizationResponse` | 자동 초기화 |
| `{model}/localization/semiautoinit` | `LocalizationSemiAutoInitRequest` | `LocalizationResponse` | 반자동 초기화 |
| `{model}/localization/randomautoinit` | `LocalizationRandomAutoInitRequest` | `LocalizationResponse` | 랜덤 자동 초기화 |
| `{model}/localization/start` | `LocalizationStartRequest` | `LocalizationResponse` | 위치추정 시작 |
| `{model}/localization/stop` | `LocalizationStopRequest` | `LocalizationResponse` | 위치추정 정지 |

### 제어 (Control) - `{model}/control/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/control/dock` | `ControlDockRequest` | `ControlResponse` | 도킹 |
| `{model}/control/undock` | `ControlUndockRequest` | `ControlResponse` | 언도킹 |
| `{model}/control/dockStop` | `ControlDockStopRequest` | `ControlResponse` | 도킹 정지 |
| `{model}/control/motor` | `ControlMotorRequest` | `ControlResponse` | 모터 On/Off |
| `{model}/control/led` | `ControlLedRequest` | `ControlResponse` | LED 제어 |
| `{model}/control/obsbox` | `ControlObsBoxRequest` | `ControlObsBoxResponse` | 장애물 박스 설정 |
| `{model}/control/getSafetyField` | `GetSafetyFieldRequest` | `GetSafetyFieldResponse` | 안전 필드 조회 |
| `{model}/control/setSafetyField` | `SetSafetyFieldRequest` | `ControlResponse` | 안전 필드 설정 |
| `{model}/control/resetSafetyFlag` | `ResetSafetyFlagRequest` | `ControlResponse` | 안전 플래그 리셋 |
| `{model}/control/getSafetyIo` | `GetSafetyIoRequest` | `GetSafetyIoResponse` | Safety I/O 조회 |
| `{model}/control/setSafetyIo` | `SetSafetyIoRequest` | `ControlResponse` | Safety I/O 설정 |
| `{model}/control/sequence` | `ControlSequenceRequest` | `ControlResponse` | 시퀀스 실행 |
| `{model}/control/setLidar` | `ControlSetLidarRequest` | `ControlSetResponse` | LiDAR 뷰 On/Off 및 주파수 |
| `{model}/control/setPath` | `ControlSetPathRequest` | `ControlSetResponse` | Path 뷰 On/Off 및 주파수 |
| `{model}/control/setStatus` | `ControlSetStatusRequest` | `ControlSetResponse` | Status On/Off 및 주파수 |
| `{model}/control/setMoveStatus` | `ControlSetMoveStatusRequest` | `ControlSetResponse` | MoveStatus On/Off 및 주파수 |

### 맵 (Map) - `{model}/map/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/map/load` | `MapLoadRequest` | `MapLoadResponse` | 맵 로드 |
| `{model}/map/getList` | `MapGetListRequest` | `MapGetListResponse` | 맵 목록 조회 |
| `{model}/map/getFile` | `MapGetFileRequest` | `MapGetFileResponse` | 맵 파일 조회 |
| `{model}/map/getCloud` | `MapGetCloudRequest` | `MapGetCloudResponse` | 맵 포인트클라우드 조회 |
| `{model}/map/getTopology` | `MapGetTopologyRequest` | `MapGetTopologyResponse` | 토폴로지 조회 |
| `{model}/map/getTile` | `MapGetTileRequest` | `MapGetTileResponse` | 맵 타일 조회 |
| `{model}/map/mapping/start` | `MappingStartRequest` | `MappingResponse` | 매핑 시작 |
| `{model}/map/mapping/stop` | `MappingStopRequest` | `MappingResponse` | 매핑 정지 |
| `{model}/map/mapping/save` | `MappingSaveRequest` | `MappingResponse` | 맵 저장 |

### 경로 (Path) - `{model}/path/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/path/request` | `PathRequest` | `PathResponse` | 경로 요청 |

### 가상 장애물 (VOBS) - `{model}/vobs/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/vobs/update` | `VobsUpdateRequest` | `VobsResponse` | 가상 장애물 업데이트 |

### 소프트웨어 (Software) - `{model}/software/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/software/update` | `SoftwareUpdateRequest` | `SoftwareUpdateResponse` | 소프트웨어 업데이트 |
| `{model}/software/getVersion` | `SoftwareGetVersionRequest` | `SoftwareGetVersionResponse` | 버전 조회 |

### 설정 (Config) - `{model}/config/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/config/setPduParam` | `SetPduParamRequest` | `SetPduParamResponse` | PDU 파라미터 설정 |
| `{model}/config/getDriveParam` | `GetDriveParamRequest` | `GetDriveParamResponse` | 드라이브 파라미터 조회 |

### 센서 (Sensor) - `{model}/sensor/*`

| Topic | Request | Response | Description |
|-------|---------|----------|-------------|
| `{model}/sensor/getInfo` | `SensorGetInfoRequest` | `SensorGetInfoResponse` | 센서 정보 조회 |
| `{model}/sensor/setInfo` | `SensorSetInfoRequest` | `SensorSetInfoResponse` | 센서 정보 설정 |
| `{model}/sensor/lidar3d/on` | `Lidar3dOnRequest` | `Lidar3dResponse` | 3D LiDAR 켜기 |
| `{model}/sensor/lidar3d/off` | `Lidar3dOffRequest` | `Lidar3dResponse` | 3D LiDAR 끄기 |

---

## Pub/Sub (주기적/실시간 데이터)

### 상태 (Status) | (pub / sub)

| Topic | Message Type | Period | Description |
|-------|--------------|--------|-------------|
| `{model}/status` | `RobotStatus` | 100ms | 로봇 상태 (센서, 배터리, 모터) |
| `{model}/moveStatus` | `MoveStatus` | 500ms | 이동 상태 (위치, 속도, 노드) |
| `{model}/result` | `Result` | 이벤트 | 결과 응답 브로드캐스트 |

### 경로 (Path) | (pub / sub)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/globalPath` | `PathData` | 전역 경로 |
| `{model}/localPath` | `PathData` | 지역 경로 |

### 센서 (Sensor) | (pub / sub)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `{model}/lidar2d` | `Lidar2DData` | 2D LiDAR 포인트 |
| `{model}/lidar3d` | `Lidar3DData` | 3D LiDAR 포인트 |
| `{model}/mappingCloud` | `MappingCloudData` | 매핑 포인트클라우드 |

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
| Sensor | `{model}/lidar*` | BestEffort | Low |




###### 우선순위 #####

### 이동 (Move) - `{model}/move/*`
### 조그 (Jog) | (pub / Sub)
### 위치추정 (Localization) - `{model}/localization/*`
### 제어 (Control) - `{model}/control/*`
    | Topic | Request | Response | Description |
    |-------|---------|----------|-------------|
    | `{model}/control/dock` | `ControlDockRequest` | `ControlResponse` | 도킹 |
    | `{model}/control/undock` | `ControlUndockRequest` | `ControlResponse` | 언도킹 |
    | `{model}/control/dockStop` | `ControlDockStopRequest` | `ControlResponse` | 도킹 정지 |
    | `{model}/control/motor` | `ControlMotorRequest` | `ControlResponse` | 모터 On/Off |
### 맵 (Map) - `{model}/map/*`
### 상태 (Status) | (pub / sub) : result는 각 
    | Topic | Message Type | Period | Description |
    |-------|--------------|--------|-------------|
    | `{model}/status` | `RobotStatus` | 100ms | 로봇 상태 (센서, 배터리, 모터) |
    | `{model}/moveStatus` | `MoveStatus` | 500ms | 이동 상태 (위치, 속도, 노드) |
    | `{model}/result` | `Result` | 이벤트 | 결과 응답 브로드캐스트 |
### 센서 (Sensor) | (pub / sub)
    | Topic | Message Type | Description |
    |-------|--------------|-------------|
    | `{model}/lidar2d` | `Lidar2DData` | 2D LiDAR 포인트 |
    | `{model}/mappingCloud` | `MappingCloudData` | 매핑 포인트클라우드 |