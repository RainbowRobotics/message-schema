# SLAM Zenoh Topics

SLAM 서비스와 AMR 서비스 간 Zenoh 통신 토픽 정의.

## Topic 구조

```
slam/
├── cmd/           # SLAM이 Subscribe (명령 수신)
├── res/           # SLAM이 Publish (응답 발행)
├── status/        # SLAM이 Publish (주기적 상태)
└── sensor/        # SLAM이 Publish (센서 데이터)
```

## 명령 (Command) - `slam/cmd/*`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `slam/cmd/move` | `MoveRequest` | 이동 명령 (goal, jog, pause, stop 등) |
| `slam/cmd/localization` | `LocalizationRequest` | 위치추정 명령 |
| `slam/cmd/mapping` | `MappingRequest` | 매핑 명령 |
| `slam/cmd/load` | `LoadRequest` | 맵/토폴로지 로드 |
| `slam/cmd/control` | `ControlRequest` | 제어 명령 (dock, motor, safety 등) |
| `slam/cmd/path` | `PathRequest` | 경로 설정 |
| `slam/cmd/vobs` | `VobsRequest` | 가상 장애물 설정 |
| `slam/cmd/sensor` | `SensorRequest` | 센서 정보 조회/설정 |
| `slam/cmd/setting` | `SettingRequest` | 파라미터 설정 |
| `slam/cmd/update` | `UpdateRequest` | 소프트웨어 업데이트 |

## 응답 (Response) - `slam/res/*`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `slam/res/move` | `MoveResponse` | 이동 명령 응답 |
| `slam/res/localization` | `LocalizationResponse` | 위치추정 명령 응답 |
| `slam/res/mapping` | `MappingResponse` | 매핑 명령 응답 |
| `slam/res/load` | `LoadResponse` | 로드 명령 응답 |
| `slam/res/control` | `ControlResponse` | 제어 명령 응답 |
| `slam/res/dock` | `DockResponse` | 도킹 명령 응답 |
| `slam/res/path` | `PathResponse` | 경로 설정 응답 |
| `slam/res/sensor` | `SensorResponse` | 센서 명령 응답 |
| `slam/res/setting` | `SettingResponse` | 설정 명령 응답 |
| `slam/res/safetyio` | `SafetyIOResponse` | Safety I/O 응답 |
| `slam/res/update` | `UpdateResponse` | 업데이트 응답 |

## 상태 (Status) - `slam/status/*`

| Topic | Message Type | Period | Description |
|-------|--------------|--------|-------------|
| `slam/status/move` | `MoveStatus` | 100ms | 이동 상태 (위치, 속도, 남은거리) |
| `slam/status/robot` | `RobotStatus` | 500ms | 로봇 상태 (모터, 배터리, 안전) |

## 센서 (Sensor) - `slam/sensor/*`

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `slam/sensor/lidar2d` | `Lidar2DData` | 2D LiDAR 포인트 |
| `slam/sensor/lidar3d` | `Lidar3DData` | 3D LiDAR 포인트 |
| `slam/sensor/global_path` | `GlobalPathData` | 전역 경로 |
| `slam/sensor/local_path` | `LocalPathData` | 지역 경로 |
| `slam/sensor/mapping_cloud` | `MappingCloudData` | 매핑 포인트클라우드 |

## Zenoh Wildcard 예시

```cpp
// 모든 SLAM 명령 구독
session.declare_subscriber("slam/cmd/*", callback);

// 모든 상태 구독
session.declare_subscriber("slam/status/*", callback);

// SLAM 전체 데이터 구독 (디버깅/모니터링)
session.declare_subscriber("slam/**", callback);
```

## QoS 권장 설정

| Topic Type | Reliability | Priority |
|------------|-------------|----------|
| `cmd/*` | Reliable | High |
| `res/*` | Reliable | High |
| `status/*` | BestEffort | Medium |
| `sensor/*` | BestEffort | Low |
