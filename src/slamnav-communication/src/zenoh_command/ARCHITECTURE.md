# comm_zenoh 아키텍처 설계 문서

## 1. 개요

SLAMNAV와 외부 시스템 간의 Zenoh 기반 통신 모듈 설계 문서.

### 핵심 원칙

| 컴포넌트 | 역할 |
|----------|------|
| **comm_zenoh.h/cpp** | robotType 관리, Session 관리, 스레드 생명주기, 모듈 주입, 범용 콜백 |
| **zenoh_command/*.cpp** | Topic 정의, FlatBuffer 데이터 처리, 모듈 직접 호출 |

### 기술 스택
- **통신**: Zenoh (C++ binding)
- **직렬화**: FlatBuffers
- **로깅**: spdlog
- **의존성**: Qt 미사용 (순수 C++)

---

## 2. 폴더 구조

```
src/slamnav-communication/
├── inc/
│   ├── comm_zenoh.h                    # 메인 헤더
│   └── flatbuffer/generated/           # FlatBuffers 자동생성 헤더
└── src/
    ├── comm_zenoh.cpp                  # 초기화 및 스레드 관리
    └── zenoh_command/
        ├── ARCHITECTURE.md             # 본 문서
        ├── topics.md                   # Topic 정의 상세
        ├── comm_zenoh_move.cpp         # Move RPC + Jog Sub + Result Pub
        ├── comm_zenoh_control.cpp      # Control RPC + Dock Result Pub
        ├── comm_zenoh_localization.cpp # Localization RPC + Result Pub
        ├── comm_zenoh_map.cpp          # Map RPC + Result Pub
        ├── comm_zenoh_setting.cpp      # Setting RPC + Result Pub
        ├── comm_zenoh_update.cpp       # Software RPC + Result Pub
        ├── comm_zenoh_status.cpp       # Status(100ms) + MoveStatus(500ms) Pub
        ├── comm_zenoh_path.cpp         # Multi RPC + Path Pub
        └── comm_zenoh_sensor.cpp       # Lidar/Cloud Pub
```

---

## 3. comm_zenoh.h/cpp 역할

### 3.1 책임 범위

| 항목 | 설명 |
|------|------|
| Zenoh Session | 생성 및 해제, 연결 상태 관리 |
| robotType | 저장 및 변경 감지 |
| 스레드 관리 | 전체/개별 시작/종료, Running 플래그 (init은 각 스레드에서 처리) |
| 모듈 주입 | `set_*_module()` 함수로 12개 외부 모듈 연결 |
| 콜백 관리 | 9개 범용 콜백 등록/호출 (구체적인 데이터 타입 미사용) |
| 경로 플래그 | global/local path 업데이트 플래그 관리 |
| 로깅 | logger.h 매크로 사용 (`log_info`, `log_debug` 등) |

### 3.2 상수 정의

```cpp
struct COMM_ZENOH_INFO
{
    static constexpr double status_send_time = 0.1;         // 100ms
    static constexpr double move_status_send_time = 0.5;    // 500ms
    static constexpr double mapping_cloud_send_time = 0.5;  // 500ms
};
```

### 3.3 클래스 구조

```cpp
class COMM_ZENOH
{
public:
    // Singleton
    static COMM_ZENOH* instance();

    // robotType 관리
    void set_robot_type(const std::string& type);
    std::string get_robot_type() const;

    // Zenoh Session 접근
    zenoh::Session& get_session();
    bool is_session_valid() const;

    // Topic 헬퍼
    std::string make_topic(const char* suffix) const;

    // 모듈 주입 (12개)
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_lidar_3d_module(LIDAR_3D* _lidar_3d);
    void set_cam_module(CAM* _cam);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_autocontrol_module(AUTOCONTROL* _ctrl);
    void set_dockcontrol_module(DOCKCONTROL* _dctrl);
    void set_localization_module(LOCALIZATION* _loc);
    void set_mapping_module(MAPPING* _mapping);

    // 모듈 접근자 (12개)
    CONFIG* get_config() const;
    LOGGER* get_logger() const;
    MOBILE* get_mobile() const;
    // ... (lidar_2d, lidar_3d, cam, unimap, obsmap, autocontrol, dockcontrol, localization, mapping)

    // 스레드 관리 - 전체
    void start_all_thread();
    void stop_all_thread();

    // 스레드 관리 - 개별 (9개 스레드)
    void start_move_thread();    void stop_move_thread();
    void start_control_thread(); void stop_control_thread();
    void start_localization_thread(); void stop_localization_thread();
    void start_map_thread();     void stop_map_thread();
    void start_setting_thread(); void stop_setting_thread();
    void start_update_thread();  void stop_update_thread();
    void start_path_thread();    void stop_path_thread();
    void start_status_thread();  void stop_status_thread();
    void start_sensor_thread();  void stop_sensor_thread();

    // 상태 플래그
    bool get_is_connected() const;

    // Running 플래그 접근자 (zenoh_command에서 사용)
    bool is_move_running() const;
    bool is_control_running() const;
    bool is_localization_running() const;
    bool is_map_running() const;
    bool is_setting_running() const;
    bool is_update_running() const;
    bool is_path_running() const;
    bool is_status_running() const;
    bool is_sensor_running() const;

    // 경로 업데이트 플래그 (zenoh_command에서 사용)
    void set_global_path_update();
    void set_local_path_update();
    bool get_global_path_update();
    bool get_local_path_update();
    void clear_global_path_update();
    void clear_local_path_update();

    // 콜백 등록/호출 (9개)
    void set_jog_callback(ZenohCallback::JogUpdate cb);
    void invoke_jog_callback(const Eigen::Vector3d& vel);
    // ... 기타 콜백

private:
    COMM_ZENOH();
    ~COMM_ZENOH();
};
```

**로깅**: `logger.h` 매크로 사용 (`log_info()`, `log_debug()` 등)

### 3.4 robotType 변경 시 동작

```cpp
void set_robot_type(const std::string& type) {
    stop_all_thread();      // 1. 모든 스레드 종료
    robot_type_ = type;     // 2. robotType 저장
    start_all_thread();     // 3. 스레드 재시작 (각 스레드에서 init)
}
```

### 3.5 콜백 타입 (ZenohCallback 네임스페이스)

범용 타입만 사용하며, 구체적인 데이터 타입은 각 zenoh_command에서 처리합니다.

```cpp
namespace ZenohCallback
{
    using JogUpdate = std::function<void(const Eigen::Vector3d& vel)>;
    using MoveStop = std::function<void()>;
    using MapBuildStart = std::function<void()>;
    using MapBuildStop = std::function<void()>;
    using MapSave = std::function<void(const std::string& map_name)>;
    using DockingStart = std::function<void()>;
    using UndockingStart = std::function<void()>;
    using DockingStop = std::function<void()>;
    using UiAllUpdate = std::function<void()>;
}
```

---

## 4. zenoh_command/*.cpp 역할

### 4.1 책임 범위

| 항목 | 설명 |
|------|------|
| Topic 정의 | 각 파일 내부에서 constexpr 또는 namespace로 선언 |
| Zenoh 리소스 | Queryable/Publisher/Subscriber 동적 등록 |
| FlatBuffer 처리 | Request 파싱, Response 생성 |
| 모듈 호출 | `get_mobile()`, `get_unimap()` 등으로 직접 호출 |
| 비즈니스 로직 | 유효성 검사, 상태 확인, 에러 처리 |

### 4.2 초기화 패턴

각 파일은 스레드 시작 시 아래 패턴으로 초기화:

```cpp
void COMM_ZENOH::move_loop() {
    // 1. Topic 정의 (파일 내부)
    constexpr const char* TOPIC_GOAL = "move/goal";
    constexpr const char* TOPIC_JOG = "move/jog";
    constexpr const char* TOPIC_RESULT = "move/result";

    // 2. robotType 대기
    while (get_robot_type().empty() && is_move_running()) {
        std::this_thread::sleep_for(100ms);
    }

    // 3. Session 유효성 확인
    if (!is_session_valid()) return;

    // 4. Topic 생성
    std::string topic_goal = get_robot_type() + "/" + TOPIC_GOAL;

    // 5. Zenoh 리소스 등록
    auto& session = get_session();
    auto pub_result = session.declare_publisher(topic_result);
    auto sub_jog = session.declare_subscriber(topic_jog, callback);
    auto q_goal = session.declare_queryable(topic_goal, callback);

    // 6. Main loop (keep alive)
    while (is_move_running()) {
        std::this_thread::sleep_for(100ms);
    }

    // 7. 스레드 종료 시 RAII로 리소스 자동 해제
}
```

---

## 5. 통신 패턴

### 5.1 RPC (Request → Response)
- **방식**: Zenoh Queryable
- **흐름**: Client Request → SLAMNAV 처리 → Response 반환
- **Topic 형식**: `{robotType}/도메인/명령`

### 5.2 Publisher (SLAMNAV → 외부)
- **방식**: Zenoh Publisher
- **용도**: 상태 정보, 센서 데이터, 경로 정보, 비동기 결과

### 5.3 Subscriber (외부 → SLAMNAV)
- **방식**: Zenoh Subscriber
- **용도**: Jog 명령 (실시간 속도 제어)

---

## 6. 데이터 처리

### 6.1 FlatBuffer Request 처리

```cpp
void handle_request(const zenoh::Query& query) {
    // 1. Payload 추출
    auto payload = query.get_payload();
    if (!payload.has_value()) {
        // 에러 응답
        return;
    }

    // 2. FlatBuffer 파싱
    auto bytes = payload->as_bytes();
    auto request = SLAMNAV::GetRequest_Move_Goal(bytes.data());

    // 3. 필드 접근
    std::string id = request->id()->str();
}
```

### 6.2 FlatBuffer Response 생성

```cpp
std::vector<uint8_t> build_response(
    const std::string& id,
    const std::string& result,
    const std::string& message
) {
    flatbuffers::FlatBufferBuilder builder(256);

    auto id_offset = builder.CreateString(id);
    auto result_offset = builder.CreateString(result);
    auto message_offset = builder.CreateString(message);

    auto response = SLAMNAV::CreateResponse_Move_Goal(
        builder, id_offset, result_offset, message_offset);

    builder.Finish(response);

    return std::vector<uint8_t>(
        builder.GetBufferPointer(),
        builder.GetBufferPointer() + builder.GetSize()
    );
}
```

---

## 7. 로깅 (logger.h 매크로 사용)

comm_msa와 동일한 방식으로 `logger.h`에 정의된 매크로를 사용합니다.

### 7.1 설정

```cpp
// comm_zenoh.cpp
namespace
{
    constexpr const char* MODULE_NAME = "ZENOH";
}
```

### 7.2 매크로 사용법

```cpp
// logger.h에 정의된 매크로
#define log_info(...)    spdlog::info("[{}] {}",    MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_warn(...)    spdlog::warn("[{}] {}",    MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_error(...)   spdlog::error("[{}] {}",   MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_debug(...)   spdlog::debug("[{}] {}",   MODULE_NAME, fmt::format(__VA_ARGS__))
#define log_critical(...) spdlog::critical("[{}] {}", MODULE_NAME, fmt::format(__VA_ARGS__))
```

### 7.3 로깅 레벨별 사용 기준

| 레벨 | 매크로 | 용도 | 예시 |
|------|--------|------|------|
| debug | `log_debug()` | 개발/디버깅용 | 모듈 설정 완료 |
| info | `log_info()` | 일반 운영 정보 | 스레드 시작/종료, 연결 상태 |
| warn | `log_warn()` | 경고 (복구 가능) | 스레드 중복 시작 시도 |
| error | `log_error()` | 에러 (복구 필요) | Session 열기 실패 |

### 7.4 사용 예시

```cpp
log_info("Constructor called");
log_info("Zenoh session opened");
log_info("robotType set to: {}", type);
log_debug("MOBILE module set");
log_warn("Move thread already running");
log_error("Failed to open Zenoh session: {}", e.what());
```

---

## 8. 스키마 파일 매핑

| 스키마 파일 | 용도 |
|-------------|------|
| `slamnav_move.fbs` | Move RPC, Jog, Result |
| `slamnav_control.fbs` | Control RPC, Dock Result |
| `slamnav_localization.fbs` | Localization RPC, Result |
| `slamnav_map.fbs` | Map RPC, Result |
| `slamnav_setting.fbs` | Setting RPC, Result |
| `slamnav_update.fbs` | Software RPC, Result |
| `slamnav_status.fbs` | Status, Move_Status |
| `slamnav_socket.fbs` | Lidar, Path |
| `slamnav_multi.fbs` | Multi-robot |

스키마 파일 위치: `schemas/slam/v1/`

---

## 9. 구현 진행 상황

| 파일 | 상태 | 리팩토링 | 설명 |
|------|------|----------|------|
| comm_zenoh.h | ✅ 완료 | ✅ 완료 | 헤더 - Session, 스레드, 모듈, 콜백 |
| comm_zenoh.cpp | ✅ 완료 | ✅ 완료 | 구현 - spdlog 로깅, 콜백 처리 |
| comm_zenoh_status.cpp | ✅ 완료 | ✅ 완료 | Status Publisher |
| comm_zenoh_moveStatus.cpp | ✅ 완료 | ✅ 완료 | MoveStatus Publisher |
| comm_zenoh_move.cpp | ✅ 완료 | ✅ 완료 | Move RPC (9개) + Jog Sub + Result Pub |
| comm_zenoh_control.cpp | ✅ 완료 | ✅ 완료 | Control RPC (16개) + Dock Result Pub |
| comm_zenoh_localization.cpp | ✅ 완료 | ✅ 완료 | Localization RPC (6개) + Result Pub |
| comm_zenoh_map.cpp | ✅ 완료 | ✅ 완료 | Map RPC (13개) + Result Pub |
| comm_zenoh_setting.cpp | ⏳ 대기 | ⏳ 대기 | Setting RPC + Result Pub |
| comm_zenoh_update.cpp | ⏳ 대기 | ⏳ 대기 | Software RPC + Result Pub |
| comm_zenoh_path.cpp | ⏳ 대기 | ⏳ 대기 | Multi RPC + Path Pub |
| comm_zenoh_sensor.cpp | ⏳ 대기 | ⏳ 대기 | Lidar/Cloud Pub |

---

## 10. 통신 흐름 다이어그램

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
│  • Topic 정의 (내부)                                             │
│  • FlatBuffer 처리                                               │
│  • 모듈 직접 호출 (mobile, unimap, loc, ctrl 등)                 │
└─────────────────────────────────────────────────────────────────┘
                            │
                            │ robotType, Session, 콜백
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                       comm_zenoh.cpp                            │
│  • robotType 관리                                               │
│  • Session 관리                                                  │
│  • 스레드 시작/종료                                              │
│  • 범용 콜백 (JogUpdate, MoveStop 등)                           │
│  • spdlog 로깅                                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 11. 참고 문서

- **topics.md**: 파일별 Topic 정의 상세
- **schemas/slam/v1/*.fbs**: FlatBuffer 스키마 파일

---

## 버전 정보

- **문서 버전**: 2.1
- **최종 수정**: 2026-02-01
- **작성 기준**: feat/zenoh_test 브랜치
