# comm_zenoh 아키텍처 설계 문서

## 1. 개요

SLAMNAV와 외부 시스템 간의 Zenoh 기반 통신 모듈 설계 문서.

### 핵심 원칙
- **comm_zenoh.cpp**: robotType 관리 및 스레드 생명주기만 담당
- **zenoh_command/*.cpp**: 각 파일에서 Zenoh 리소스(Queryable/Publisher/Subscriber) 동적 등록
- **데이터 형식**: FlatBuffers 사용 (`schemas/slam/v1/*.fbs`)

---

## 2. 폴더 구조

```
src/slamnav-communication/
├── inc/
│   ├── comm_zenoh.h                    # 메인 헤더
│   └── flatbuffer/generated/           # FlatBuffers 자동생성 헤더
│       ├── slamnav_move_generated.h
│       ├── slamnav_status_generated.h
│       ├── slamnav_control_generated.h
│       ├── slamnav_map_generated.h
│       ├── slamnav_localization_generated.h
│       ├── slamnav_setting_generated.h
│       ├── slamnav_update_generated.h
│       ├── slamnav_socket_generated.h
│       └── slamnav_multi_generated.h
└── src/
    ├── comm_zenoh.cpp                  # 초기화 및 스레드 관리
    └── zenoh_command/
        ├── comm_zenoh_move.cpp         # Move RPC + Jog Sub + Result Pub
        ├── comm_zenoh_control.cpp      # Control RPC
        ├── comm_zenoh_localization.cpp # Localization RPC + Result Pub
        ├── comm_zenoh_map.cpp          # Map RPC + Result Pub
        ├── comm_zenoh_setting.cpp      # Setting RPC + Result Pub
        ├── comm_zenoh_update.cpp       # Software RPC + Result Pub
        ├── comm_zenoh_status.cpp       # Status Pub (100ms)
        ├── comm_zenoh_moveStatus.cpp   # MoveStatus Pub (500ms)
        ├── comm_zenoh_path.cpp         # Multi RPC + Path Pub
        └── comm_zenoh_sensor.cpp       # Lidar/Cloud Pub
```

---

## 3. comm_zenoh.cpp 역할

### 3.1 책임 범위
| 항목 | 설명 |
|------|------|
| Zenoh Session | 생성 및 해제 |
| robotType | 저장 및 변경 감지 |
| 스레드 관리 | 시작/종료 (init은 각 스레드에서 처리) |
| 모듈 주입 | set_*_module() 함수로 외부 모듈 연결 |

### 3.2 robotType 변경 시 동작
```cpp
void set_robot_type(const std::string& type) {
    stop_all_thread();      // 1. 모든 스레드 종료
    robot_type_ = type;     // 2. robotType 저장
    start_all_thread();     // 3. 스레드 재시작 (각 스레드에서 init)
}
```

### 3.3 주요 함수
```cpp
// robotType 관리
void set_robot_type(const std::string& type);
std::string get_robot_type();

// 스레드 관리
void start_all_thread();
void stop_all_thread();

// 모듈 주입
void set_mobile_module(MOBILE* _mobile);
void set_localization_module(LOCALIZATION* _loc);
void set_mapping_module(MAPPING* _mapping);
void set_autocontrol_module(AUTOCONTROL* _ctrl);
// ... 기타 모듈
```

---

## 4. 통신 패턴

### 4.1 RPC (Request → Response)
- **방식**: Zenoh Queryable
- **흐름**: Client가 Request 전송 → SLAMNAV가 처리 → Response 반환
- **Topic 형식**: `{robotType}/도메인/명령`

### 4.2 Publisher (SLAMNAV → 외부)
- **방식**: Zenoh Publisher
- **흐름**: SLAMNAV가 주기적/이벤트 기반으로 데이터 발행
- **용도**: 상태 정보, 센서 데이터, 경로 정보

### 4.3 Subscriber (외부 → SLAMNAV)
- **방식**: Zenoh Subscriber
- **흐름**: 외부에서 발행한 데이터를 SLAMNAV가 수신
- **용도**: Jog 명령 (실시간 속도 제어)

### 4.4 Result Publisher
- **방식**: Zenoh Publisher
- **흐름**: RPC 처리 완료 후 결과(id, success/fail) 발행
- **용도**: 비동기 작업 결과 알림

---

## 5. 파일별 Topic 정의

### 5.1 comm_zenoh_move.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/move/goal` | Request_Move_Goal → Response_Move_Goal |
| RPC | `{robotType}/move/target` | Request_Move_Target → Response_Move_Target |
| RPC | `{robotType}/move/stop` | Request_Move_Stop → Response_Move_Stop |
| RPC | `{robotType}/move/pause` | Request_Move_Pause → Response_Move_Pause |
| RPC | `{robotType}/move/resume` | Request_Move_Resume → Response_Move_Resume |
| RPC | `{robotType}/move/xLinear` | Request_Move_XLinear → Response_Move_XLinear |
| RPC | `{robotType}/move/circular` | Request_Move_Circular → Response_Move_Circular |
| RPC | `{robotType}/move/rotate` | Request_Move_Rotate → Response_Move_Rotate |
| Sub | `{robotType}/move/jog` | Move_Jog |
| Pub | `{robotType}/move/result` | Move_Result (id, result, message) |

### 5.2 comm_zenoh_control.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/control/getSafetyField` | Request → Response |
| RPC | `{robotType}/control/setSafetyField` | Request → Response |
| RPC | `{robotType}/control/getSafetyFlag` | Request → Response |
| RPC | `{robotType}/control/setSafetyFlag` | Request → Response |
| RPC | `{robotType}/control/getSafetyIo` | Request → Response |
| RPC | `{robotType}/control/setSafetyIo` | Request → Response |
| RPC | `{robotType}/control/dock` | Request_Dock → Response_Dock |
| RPC | `{robotType}/control/chargeTrigger` | Request → Response |
| RPC | `{robotType}/control/getObsBox` | Request → Response |
| RPC | `{robotType}/control/setObsBox` | Request → Response |
| RPC | `{robotType}/control/led` | Request_Led → Response_Led |
| RPC | `{robotType}/control/motor` | Request_Motor → Response_Motor |
| RPC | `{robotType}/control/jog` | Request_Jog → Response_Jog |
| RPC | `{robotType}/control/sensor` | Request_Sensor → Response_Sensor |
| RPC | `{robotType}/control/path` | Request_Path → Response_Path |
| RPC | `{robotType}/control/detect` | Request_Detect → Response_Detect |

### 5.3 comm_zenoh_localization.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/localization/init` | Request → Response |
| RPC | `{robotType}/localization/autoinit` | Request → Response |
| RPC | `{robotType}/localization/semiautoinit` | Request → Response |
| RPC | `{robotType}/localization/randominit` | Request → Response |
| RPC | `{robotType}/localization/start` | Request → Response |
| RPC | `{robotType}/localization/stop` | Request → Response |
| Pub | `{robotType}/localization/result` | Localization_Result |

### 5.4 comm_zenoh_map.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/map/getList` | Request → Response |
| RPC | `{robotType}/map/getCurrent` | Request → Response |
| RPC | `{robotType}/map/load` | Request → Response |
| RPC | `{robotType}/map/delete` | Request → Response |
| RPC | `{robotType}/map/getFile` | Request → Response |
| RPC | `{robotType}/map/getCloud` | Request → Response |
| RPC | `{robotType}/map/setCloud` | Request → Response |
| RPC | `{robotType}/map/getTopology` | Request → Response |
| RPC | `{robotType}/map/setTopology` | Request → Response |
| RPC | `{robotType}/map/mapping/start` | Request → Response |
| RPC | `{robotType}/map/mapping/stop` | Request → Response |
| RPC | `{robotType}/map/mapping/save` | Request → Response |
| Pub | `{robotType}/map/result` | Map_Result |

### 5.5 comm_zenoh_setting.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/setting/getSensorIndex` | Request → Response |
| RPC | `{robotType}/setting/setSensorIndex` | Request → Response |
| RPC | `{robotType}/setting/setSensorOn` | Request → Response |
| RPC | `{robotType}/setting/getSensorOff` | Request → Response |
| RPC | `{robotType}/setting/getPduParam` | Request → Response |
| RPC | `{robotType}/setting/setPduParam` | Request → Response |
| RPC | `{robotType}/setting/getDriveParam` | Request → Response |
| Pub | `{robotType}/setting/result` | Setting_Result |

### 5.6 comm_zenoh_update.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/software/update` | Request_Update → Response_Update |
| RPC | `{robotType}/software/getVersion` | Request → Response |
| Pub | `{robotType}/software/result` | Update_Result |

### 5.7 comm_zenoh_status.cpp

| 패턴 | Topic | 주기 | 스키마 |
|------|-------|------|--------|
| Pub | `{robotType}/status` | 100ms | Status |

### 5.8 comm_zenoh_moveStatus.cpp

| 패턴 | Topic | 주기 | 스키마 |
|------|-------|------|--------|
| Pub | `{robotType}/moveStatus` | 500ms | Move_Status |

### 5.9 comm_zenoh_path.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/multi/path` | Request_Path → Response_Path |
| RPC | `{robotType}/multi/vobs` | Request_Vobs → Response_Vobs |
| Pub | `{robotType}/globalPath` | Global_Path |
| Pub | `{robotType}/localPath` | Local_Path |

### 5.10 comm_zenoh_sensor.cpp

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| Pub | `{robotType}/lidar2d` | Lidar_2D |
| Pub | `{robotType}/lidar3d` | Lidar_3D |
| Pub | `{robotType}/mappingCloud` | Mapping_Cloud |

---

## 6. 제외 항목 (구현하지 않음)

| Topic | 사유 |
|-------|------|
| `{robotType}/move/stateChange` | 주석 처리 |
| `{robotType}/control/dock/stateChange` | 주석 처리 |

---

## 7. Result 메시지 구조

모든 Result는 동일한 구조를 따름:
```flatbuffers
table [Domain]_Result {
    id: string;        // RPC 요청의 id 값 그대로 사용
    result: string;    // "success" 또는 "fail"
    message: string;   // 상세 메시지 (선택)
}
```

---

## 8. 스키마 파일 매핑

| 스키마 파일 | 용도 |
|-------------|------|
| `slamnav_move.fbs` | Move 관련 Request/Response, Jog, Result |
| `slamnav_control.fbs` | Control 관련 Request/Response |
| `slamnav_localization.fbs` | Localization Request/Response, Result |
| `slamnav_map.fbs` | Map 관련 Request/Response, Result |
| `slamnav_setting.fbs` | Setting Request/Response, Result |
| `slamnav_update.fbs` | Software Update Request/Response, Result |
| `slamnav_status.fbs` | Status, Move_Status |
| `slamnav_socket.fbs` | Lidar_2D, Lidar_3D, Mapping_Cloud, Path |
| `slamnav_multi.fbs` | Multi-robot Path, Vobs |

---

## 9. 통신 흐름 다이어그램

```
┌─────────────────────────────────────────────────────────────────┐
│                         Client (FMS/UI)                         │
└─────────────────────────────────────────────────────────────────┘
        │                    │                    │
        │ RPC Request        │ Subscribe          │ Publish
        │ (Queryable)        │ (status, etc.)     │ (jog)
        ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                           Zenoh                                 │
└─────────────────────────────────────────────────────────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                      zenoh_command/*.cpp                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │   RPC 처리    │  │  Publisher   │  │  Subscriber  │          │
│  │  (Queryable) │  │ (status,etc) │  │    (jog)     │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
│         │                 │                 │                   │
│         └─────────────────┴─────────────────┘                   │
│                           │                                     │
│                    robotType 참조                               │
│                           │                                     │
└───────────────────────────┼─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                       comm_zenoh.cpp                            │
│  • robotType 저장                                               │
│  • 스레드 시작/종료                                              │
│  • robotType 변경 시 → stop_all → start_all                     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 10. zenoh_command/*.cpp 초기화 패턴

각 파일은 스레드 시작 시 아래 패턴으로 초기화:

```cpp
void CommZenoh::move_loop() {
    // 1. robotType으로 topic 생성
    std::string topic_goal = get_robot_type() + "/move/goal";
    std::string topic_jog = get_robot_type() + "/move/jog";
    std::string topic_result = get_robot_type() + "/move/result";

    // 2. Zenoh 리소스 등록
    auto queryable_goal = session_->declare_queryable(topic_goal, ...);
    auto subscriber_jog = session_->declare_subscriber(topic_jog, ...);
    auto publisher_result = session_->declare_publisher(topic_result);

    // 3. 메인 루프
    while (running_) {
        // 처리 로직
    }

    // 4. 스레드 종료 시 리소스 자동 해제 (RAII)
}
```

---

## 11. 구현 진행 상황

| 파일 | 상태 | 설명 |
|------|------|------|
| **comm_zenoh.h** | ✅ 완료 | 헤더 파일 - robotType 관리, 스레드 관리, 모듈 접근자 |
| **comm_zenoh.cpp** | ✅ 완료 | 메인 구현 - Session 관리, 스레드 시작/종료 |
| **comm_zenoh_status.cpp** | ✅ 완료 | Status(100ms) + MoveStatus(500ms) Publisher |
| **comm_zenoh_move.cpp** | ✅ 완료 | Move RPC + Jog Subscriber + Result Publisher |
| **comm_zenoh_control.cpp** | ✅ 완료 | Control RPC (16개) |
| **comm_zenoh_localization.cpp** | ✅ 완료 | Localization RPC (6개) + Result Publisher |
| **comm_zenoh_map.cpp** | ✅ 완료 | Map RPC (12개) + Result Publisher |
| comm_zenoh_setting.cpp | ✅ 완료 | Setting RPC (7개) + Result Publisher |
| comm_zenoh_update.cpp | ✅ 완료 | Software RPC (2개) + Result Publisher |
| comm_zenoh_path.cpp | ⏳ 대기 | Multi RPC + globalPath/localPath Publisher |
| comm_zenoh_sensor.cpp | ⏳ 대기 | lidar2d/lidar3d/mappingCloud Publisher |

---

## 12. 구현 상세 - comm_zenoh_status.cpp

### 12.1 역할
- **status_loop()**: Status와 MoveStatus를 주기적으로 발행하는 스레드 루프

### 12.2 Publisher Topics

| Topic | 주기 | 스키마 | 내용 |
|-------|------|--------|------|
| `{robotType}/status` | 100ms | Status (slamnav_status.fbs) | IMU, Motor, Power, RobotState, SafetyIO, Setting, Map |
| `{robotType}/moveStatus` | 500ms | Move_Status (slamnav_status.fbs) | cur_node, goal_node, move_state, pose, vel |

### 12.3 데이터 수집 모듈

| 데이터 | 모듈 | 함수 |
|--------|------|------|
| IMU, Motor, Power | MOBILE | `get_status()`, `get_imu()` |
| Localization 상태 | LOCALIZATION | `get_cur_loc_state()`, `get_cur_ieir()`, `get_cur_pose()` |
| 도킹 상태 | DOCKCONTROL | `get_dock_state()` |
| 맵 정보 | UNIMAP | `get_is_loaded()`, `get_map_path()` |
| 로봇 설정 | CONFIG | `get_robot_type()`, `get_robot_model()` |
| 이동 상태 | AUTOCONTROL | `get_is_moving()`, `get_is_pause()`, `get_cur_node_id()` 등 |

### 12.4 코드 구조

```cpp
// Helper 함수 (anonymous namespace)
get_charge_state_string()     // 충전 상태 → 문자열
get_map_status_string()       // 맵 상태 → 문자열
get_auto_move_state_string()  // 자동 이동 상태 → 문자열
get_dock_move_state_string()  // 도킹 이동 상태 → 문자열

// Publisher 함수 (static)
publish_status()              // Status 메시지 생성 및 발행
publish_move_status()         // MoveStatus 메시지 생성 및 발행

// 스레드 루프 (COMM_ZENOH 멤버)
COMM_ZENOH::status_loop()     // 메인 루프 - 타이밍 관리 및 발행
```

### 12.5 타이밍 관리

```cpp
// status_loop() 내부
auto last_status_time = std::chrono::steady_clock::now();
auto last_move_status_time = std::chrono::steady_clock::now();

while (is_status_running_.load()) {
    auto now = std::chrono::steady_clock::now();

    // Status 발행 (100ms)
    if (now - last_status_time >= 100ms) {
        publish_status(this);
        last_status_time = now;
    }

    // MoveStatus 발행 (500ms)
    if (now - last_move_status_time >= 500ms) {
        publish_move_status(this);
        last_move_status_time = now;
    }

    std::this_thread::sleep_for(10ms);  // CPU 사용량 최소화
}
```

---

## 13. 구현 상세 - comm_zenoh_move.cpp

### 13.1 역할
- **move_loop()**: Move 도메인의 모든 RPC, Subscriber, Publisher 처리

### 13.2 Topics

| 패턴 | Topic | 스키마 |
|------|-------|--------|
| RPC | `{robotType}/move/goal` | Request_Move_Goal → Response_Move_Goal |
| RPC | `{robotType}/move/target` | Request_Move_Target → Response_Move_Target |
| RPC | `{robotType}/move/stop` | Request_Move_Stop → Response_Move_Stop |
| RPC | `{robotType}/move/pause` | Request_Move_Pause → Response_Move_Pause |
| RPC | `{robotType}/move/resume` | Request_Move_Resume → Response_Move_Resume |
| RPC | `{robotType}/move/xLinear` | Request_Move_XLinear → Response_Move_XLinear |
| RPC | `{robotType}/move/circular` | Request_Move_Circular → Response_Move_Circular |
| RPC | `{robotType}/move/rotate` | Request_Move_Rotate → Response_Move_Rotate |
| Sub | `{robotType}/move/jog` | Move_Jog |
| Pub | `{robotType}/move/result` | Move_Result |

### 13.3 코드 구조

```cpp
// Helper 함수 (anonymous namespace)
build_move_result()       // Move_Result FlatBuffer 생성
build_response_goal()     // Response_Move_Goal FlatBuffer 생성
build_response_target()   // Response_Move_Target FlatBuffer 생성
build_response_stop()     // Response_Move_Stop FlatBuffer 생성
build_response_pause()    // Response_Move_Pause FlatBuffer 생성
build_response_resume()   // Response_Move_Resume FlatBuffer 생성
build_response_xlinear()  // Response_Move_XLinear FlatBuffer 생성
build_response_circular() // Response_Move_Circular FlatBuffer 생성
build_response_rotate()   // Response_Move_Rotate FlatBuffer 생성

// 스레드 루프 (COMM_ZENOH 멤버)
COMM_ZENOH::move_loop()   // 메인 루프 - Zenoh 리소스 등록 및 처리
```

### 13.4 Zenoh 리소스 등록 순서

```cpp
void COMM_ZENOH::move_loop() {
    // 1. robotType 대기
    while (get_robot_type().empty()) { ... }

    // 2. Session 유효성 확인
    if (!is_session_valid()) return;

    // 3. Topic 생성
    std::string topic_goal = make_topic(ZENOH_TOPIC::MOVE_GOAL);
    // ... 나머지 topic 생성

    // 4. Result Publisher 등록
    auto pub_result = session.declare_publisher(topic_result);

    // 5. Jog Subscriber 등록
    auto sub_jog = session.declare_subscriber(topic_jog, callback);

    // 6. RPC Queryable 등록 (8개)
    auto q_goal = session.declare_queryable(topic_goal, callback);
    // ... 나머지 queryable 등록

    // 7. Main loop (keep alive)
    while (is_move_running_.load()) {
        std::this_thread::sleep_for(100ms);
    }

    // 8. 스레드 종료 시 RAII로 리소스 자동 해제
}
```

### 13.5 RPC 처리 흐름

```
Client Request (FlatBuffers)
       │
       ▼
Zenoh Queryable Callback
       │
       ├─→ Payload 파싱 (GetRequest_Move_*)
       │
       ├─→ 유효성 검사
       │   • 모듈 존재 확인
       │   • 맵 로드 확인
       │   • 위치 추정 확인
       │   • 범위/충돌 확인 (target)
       │
       ├─→ Response 생성 및 전송
       │   query.reply(Response_Move_*)
       │
       ├─→ 내부 시그널 발행
       │   Q_EMIT ctrl->signal_move(msg)
       │
       └─→ Result 발행
           pub_result.put(Move_Result)
```

### 13.6 Jog Subscriber 처리

```cpp
auto sub_jog = session.declare_subscriber(
    topic_jog,
    [this](const zenoh::Sample& sample) {
        auto jog = SLAMNAV::GetMove_Jog(payload);
        double vx = jog->vx();
        double vy = jog->vy();
        double wz = jog->wz() * D2R;
        Q_EMIT signal_mobile_jog_update(Eigen::Vector3d(vx, vy, wz));
    }
);
```

### 13.7 RPC별 검증 로직

| RPC | 검증 항목 |
|-----|----------|
| **goal** | 모듈 존재, 맵 로드 상태, 위치 추정 상태, 노드 존재 여부 |
| **target** | 모듈 존재, 맵 로드 상태, 위치 추정 상태, 좌표 범위, 충돌 검사 |
| **stop** | 없음 (항상 accept) |
| **pause** | 없음 (항상 accept) |
| **resume** | 없음 (항상 accept) |
| **xLinear** | target ≤ 10.0m, speed ≤ 1.5m/s |
| **circular** | target ≤ 360°, speed ≤ 60°/s |
| **rotate** | target ≤ 360°, speed ≤ 60°/s |

### 13.8 사용 모듈 및 Qt 시그널

| 모듈 | 용도 |
|------|------|
| `mobile` | Jog 속도 업데이트, 정지 명령 |
| `unimap` | 노드 조회, 맵 경계 확인 |
| `loc` | 위치 추정 상태, 현재 위치 조회 |
| `obsmap` | 충돌 검사 |
| `ctrl` | 이동 시그널 발행, pause/resume 상태 설정 |

| Qt 시그널 | 발생 조건 | 수신 슬롯 |
|-----------|----------|----------|
| `signal_mobile_jog_update` | Jog 메시지 수신 시 | MOBILE::slot_jog_update |
| `ctrl->signal_move` | goal/target 요청 accept 시 | AUTOCONTROL |
| `signal_auto_profile_move` | xLinear/circular/rotate accept 시 | AUTOCONTROL |
| `signal_auto_move_stop` | stop 요청 시 | AUTOCONTROL |

### 13.9 에러 Response 메시지

| 에러 상황 | result | message |
|----------|--------|---------|
| Payload 없음 | reject | no payload |
| FlatBuffer 파싱 실패 | reject | invalid request |
| 필수 모듈 없음 | reject | module not ready |
| 맵 미로드 | reject | map not loaded |
| 위치 추정 안됨 | reject | not localized |
| 노드 찾기 실패 | reject | goal not found |
| 좌표 범위 초과 | reject | out of bounds |
| 충돌 감지 | reject | collision |
| 파라미터 범위 초과 | reject | value out of range |

---

## 14. 버전 정보

- **문서 버전**: 1.3
- **최종 수정**: 2026-01-28
- **작성 기준**: feat/zenoh_test 브랜치
