# COMM_ZENOH 아키텍처 문서

이 문서는 `comm_zenoh.h`, `comm_zenoh.cpp`, `comm_zenoh_moveStatus.cpp`의 구조와 데이터 처리 방식을 정리한 것입니다.

---

## 1. 클래스 개요

### 1.1 COMM_ZENOH 클래스

Zenoh 통신을 관리하는 **싱글톤 클래스**입니다.

```cpp
class COMM_ZENOH
{
public:
    static COMM_ZENOH* instance();  // 싱글톤 접근
    // ...
private:
    COMM_ZENOH();   // private 생성자
    ~COMM_ZENOH();
};
```

**싱글톤 구현:**
```cpp
COMM_ZENOH* COMM_ZENOH::instance()
{
    static COMM_ZENOH* inst = nullptr;
    if (!inst)
    {
        inst = new COMM_ZENOH();
    }
    return inst;
}
```

### 1.2 복사/이동 금지

```cpp
COMM_ZENOH(const COMM_ZENOH&) = delete;
COMM_ZENOH& operator=(const COMM_ZENOH&) = delete;
COMM_ZENOH(COMM_ZENOH&&) = delete;
COMM_ZENOH& operator=(COMM_ZENOH&&) = delete;
```

---

## 2. 상수 정의

### 2.1 COMM_ZENOH_INFO 구조체

```cpp
struct COMM_ZENOH_INFO
{
    static constexpr double status_send_time = 0.1;         // 100ms - Status 발행 주기
    static constexpr double move_status_send_time = 0.5;    // 500ms - MoveStatus 발행 주기
    static constexpr double mapping_cloud_send_time = 0.5;  // 500ms - MappingCloud 발행 주기
};
```

### 2.2 모듈별 상수 (익명 네임스페이스)

각 cpp 파일에서 익명 네임스페이스로 모듈 이름과 주기 정의:

```cpp
namespace
{
    constexpr const char* MODULE_NAME = "MOVE_STATUS";
    constexpr int MOVE_STATUS_INTERVAL_MS = 500;  // 500ms
}
```

---

## 3. Zenoh 세션 관리

### 3.1 세션 저장 방식

```cpp
private:
    std::optional<zenoh::Session> session_;     // optional로 세션 관리
    mutable std::shared_mutex session_mtx_;     // 읽기/쓰기 락
```

### 3.2 세션 열기/닫기

```cpp
void COMM_ZENOH::open_session()
{
    std::unique_lock<std::shared_mutex> lock(session_mtx_);  // 쓰기 락
    try
    {
        zenoh::Config config = zenoh::Config::create_default();
        session_ = zenoh::Session::open(std::move(config));
        is_connected_ = true;
    }
    catch (const zenoh::ZException& e)
    {
        is_connected_ = false;
    }
}

void COMM_ZENOH::close_session()
{
    std::unique_lock<std::shared_mutex> lock(session_mtx_);
    if (session_.has_value())
    {
        session_.reset();
        is_connected_ = false;
    }
}
```

### 3.3 세션 접근

```cpp
zenoh::Session& COMM_ZENOH::get_session()
{
    std::shared_lock<std::shared_mutex> lock(session_mtx_);  // 읽기 락
    if (!session_.has_value())
    {
        throw std::runtime_error("Zenoh session is not initialized");
    }
    return session_.value();
}

bool COMM_ZENOH::is_session_valid() const
{
    std::shared_lock<std::shared_mutex> lock(session_mtx_);
    return session_.has_value();
}
```

---

## 4. Robot Type 관리

### 4.1 저장 및 동기화

```cpp
private:
    std::string robot_type_;
    mutable std::shared_mutex robot_type_mtx_;
```

### 4.2 설정 시 스레드 재시작

```cpp
void COMM_ZENOH::set_robot_type(const std::string& type)
{
    stop_all_thread();   // 기존 스레드 정지
    {
        std::unique_lock<std::shared_mutex> lock(robot_type_mtx_);
        robot_type_ = type;
    }
    start_all_thread();  // 새 스레드 시작
}
```

### 4.3 토픽 생성

```cpp
std::string COMM_ZENOH::make_topic(const char* suffix) const
{
    std::shared_lock<std::shared_mutex> lock(robot_type_mtx_);
    if (robot_type_.empty())
    {
        return std::string(suffix);
    }
    return robot_type_ + "/" + suffix;  // 예: "AMR001/moveStatus"
}
```

---

## 5. 모듈 의존성 주입

### 5.1 지원 모듈 목록

| 모듈 | 타입 | 용도 |
|------|------|------|
| CONFIG | `CONFIG*` | 설정 관리 |
| LOGGER | `LOGGER*` | 로깅 |
| MOBILE | `MOBILE*` | 모바일 플랫폼 제어 |
| LIDAR_2D | `LIDAR_2D*` | 2D 라이다 |
| LIDAR_3D | `LIDAR_3D*` | 3D 라이다 |
| CAM | `CAM*` | 카메라 |
| UNIMAP | `UNIMAP*` | 맵 관리 |
| OBSMAP | `OBSMAP*` | 장애물 맵 |
| AUTOCONTROL | `AUTOCONTROL*` | 자동 제어 |
| DOCKCONTROL | `DOCKCONTROL*` | 도킹 제어 |
| LOCALIZATION | `LOCALIZATION*` | 위치 추정 |
| MAPPING | `MAPPING*` | 맵핑 |

### 5.2 설정/조회 패턴

```cpp
// Setter
void COMM_ZENOH::set_mobile_module(MOBILE* _mobile)
{
    if (_mobile)
    {
        mobile = _mobile;
        log_debug("MOBILE module set");
    }
}

// Getter
MOBILE* get_mobile() const { return mobile; }
```

---

## 6. 스레드 관리

### 6.1 스레드 목록

| 스레드 | 플래그 | 루프 함수 | 구현 파일 |
|--------|--------|----------|----------|
| move | `is_move_running_` | `move_loop()` | comm_zenoh_move.cpp |
| control | `is_control_running_` | `control_loop()` | comm_zenoh_control.cpp |
| localization | `is_localization_running_` | `localization_loop()` | comm_zenoh_localization.cpp |
| map | `is_map_running_` | `map_loop()` | comm_zenoh_map.cpp |
| setting | `is_setting_running_` | `setting_loop()` | (placeholder) |
| update | `is_update_running_` | `update_loop()` | (placeholder) |
| path | `is_path_running_` | `path_loop()` | (placeholder) |
| status | `is_status_running_` | `status_loop()` | comm_zenoh_status.cpp |
| move_status | `is_move_status_running_` | `move_status_loop()` | comm_zenoh_moveStatus.cpp |
| sensor | `is_sensor_running_` | `sensor_loop()` | (placeholder) |

### 6.2 스레드 시작/정지 패턴

```cpp
void COMM_ZENOH::start_move_thread()
{
    // 중복 시작 방지
    if (is_move_running_.load())
    {
        log_warn("Move thread already running");
        return;
    }

    is_move_running_ = true;
    move_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::move_loop, this);
    log_info("Move thread started");
}

void COMM_ZENOH::stop_move_thread()
{
    // 실행 중이 아니면 무시
    if (!is_move_running_.load())
    {
        return;
    }

    is_move_running_ = false;
    if (move_thread_ && move_thread_->joinable())
    {
        move_thread_->join();
        move_thread_.reset();
        log_info("Move thread stopped");
    }
}
```

### 6.3 전체 스레드 제어

```cpp
void COMM_ZENOH::start_all_thread()
{
    start_move_thread();
    start_control_thread();
    start_localization_thread();
    start_map_thread();
    start_setting_thread();
    start_update_thread();
    start_path_thread();
    start_status_thread();
    start_move_status_thread();
    start_sensor_thread();
}

void COMM_ZENOH::stop_all_thread()
{
    stop_move_thread();
    stop_control_thread();
    // ... 모든 스레드 정지
}
```

---

## 7. 콜백 시스템

### 7.1 콜백 타입 정의 (ZenohCallback 네임스페이스)

```cpp
namespace ZenohCallback
{
    // Move 관련
    using JogUpdate = std::function<void(const Eigen::Vector3d& vel)>;
    using MoveStop = std::function<void()>;

    // Map 관련
    using MapBuildStart = std::function<void()>;
    using MapBuildStop = std::function<void()>;
    using MapSave = std::function<void(const std::string& map_name)>;

    // Docking 관련
    using DockingStart = std::function<void()>;
    using UndockingStart = std::function<void()>;
    using DockingStop = std::function<void()>;

    // UI 관련
    using UiAllUpdate = std::function<void()>;
}
```

### 7.2 콜백 등록/호출 패턴

```cpp
// 콜백 저장
private:
    ZenohCallback::JogUpdate jog_callback_;
    std::mutex callback_mtx_;

// 등록
void COMM_ZENOH::set_jog_callback(ZenohCallback::JogUpdate cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    jog_callback_ = std::move(cb);
}

// 호출 (zenoh_command에서 사용)
void COMM_ZENOH::invoke_jog_callback(const Eigen::Vector3d& vel)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (jog_callback_)
    {
        jog_callback_(vel);
    }
}
```

---

## 8. 스레드 루프 구현 패턴

### 8.1 move_status_loop() 예시

```cpp
void COMM_ZENOH::move_status_loop()
{
    log_info("move_status_loop started");

    // 1. robotType 설정 대기
    while (is_move_status_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 2. 조기 종료 체크
    if (!is_move_status_running_.load())
    {
        log_info("move_status_loop ended (stopped before init)");
        return;
    }

    log_info("move_status_loop initialized with robotType: {}", get_robot_type());

    // 3. 주기적 실행
    auto last_time = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(MOVE_STATUS_INTERVAL_MS);

    while (is_move_status_running_.load())
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_time >= interval)
        {
            publish_move_status(this);  // 실제 작업
            last_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // CPU 부하 방지
    }

    log_info("move_status_loop ended");
}
```

### 8.2 루프 구현 체크리스트

1. **시작 로그**: `log_info("xxx_loop started")`
2. **robotType 대기**: `while (running && robot_type.empty())`
3. **조기 종료 체크**: 초기화 전 정지 요청 처리
4. **초기화 완료 로그**: robotType 포함
5. **주기적 실행**: `interval` 기반 타이밍
6. **CPU 부하 방지**: 짧은 sleep (10ms)
7. **종료 로그**: `log_info("xxx_loop ended")`

---

## 9. FlatBuffers 데이터 처리

### 9.1 발행 함수 패턴

```cpp
static void publish_move_status(COMM_ZENOH* zenoh)
{
    // 1. 연결 상태 확인
    if (!zenoh->get_is_connected()) return;
    if (!zenoh->is_session_valid()) return;

    // 2. 필요한 모듈 가져오기
    AUTOCONTROL* ctrl = zenoh->get_autocontrol();
    MOBILE* mobile = zenoh->get_mobile();
    // ...

    // 3. 모듈 유효성 확인
    if (!ctrl || !mobile || !unimap || !dctrl || !loc) return;

    try
    {
        // 4. FlatBufferBuilder 생성
        flatbuffers::FlatBufferBuilder fbb(1024);

        // 5. 데이터 수집 및 변환
        const MOBILE_POSE mo = mobile->get_pose();
        // ...

        // 6. FlatBuffer 테이블 생성 (내부에서 외부 순서)
        auto move_state = SLAMNAV::CreateStatusMoveState(fbb, ...);
        SLAMNAV::StatusPose pose(...);
        auto move_status = SLAMNAV::CreateMoveStatus(fbb, ...);

        // 7. 빌드 완료
        fbb.Finish(move_status);

        // 8. 바이너리 추출
        const uint8_t* data = fbb.GetBufferPointer();
        size_t size = fbb.GetSize();
        std::string payload(reinterpret_cast<const char*>(data), size);

        // 9. Zenoh 발행
        std::string topic = zenoh->make_topic("moveStatus");
        zenoh->get_session().put(zenoh::KeyExpr(topic), payload);
    }
    catch (const std::exception& e)
    {
        log_error("publish_move_status error: {}", e.what());
    }
}
```

### 9.2 FlatBuffers 빌드 순서

1. **struct** (값 타입): 직접 생성
   ```cpp
   SLAMNAV::StatusPose pose(x, y, rz);
   ```

2. **table** (참조 타입): `Create*` 함수 사용
   ```cpp
   auto move_state = SLAMNAV::CreateStatusMoveState(fbb,
       fbb.CreateString("stop"),
       fbb.CreateString("none"),
       // ...
   );
   ```

3. **문자열**: `fbb.CreateString()` 사용
   ```cpp
   fbb.CreateString(ctrl->get_auto_state().toStdString())
   ```

4. **빌드 완료**: `fbb.Finish(root_table)`

### 9.3 struct vs table 전달 방식

| 타입 | 생성 방식 | Create* 함수 전달 |
|------|----------|------------------|
| struct | 직접 생성 | 포인터 (`&pose`) |
| table | Create* 함수 | Offset 값 그대로 |

```cpp
auto move_status = SLAMNAV::CreateMoveStatus(fbb,
    move_state,  // table offset
    &pose,       // struct 포인터
    &vel         // struct 포인터
);
```

---

## 10. Path Update 플래그

### 10.1 원자적 플래그

```cpp
private:
    std::atomic<bool> is_global_path_update_{false};
    std::atomic<bool> is_local_path_update_{false};
```

### 10.2 설정/조회/클리어

```cpp
void set_global_path_update()   { is_global_path_update_ = true; }
void set_local_path_update()    { is_local_path_update_ = true; }
bool get_global_path_update()   { return is_global_path_update_.load(); }
bool get_local_path_update()    { return is_local_path_update_.load(); }
void clear_global_path_update() { is_global_path_update_ = false; }
void clear_local_path_update()  { is_local_path_update_ = false; }
```

---

## 11. 동기화 전략 요약

| 리소스 | 동기화 방식 | 용도 |
|--------|------------|------|
| `session_` | `std::shared_mutex` | 세션 읽기/쓰기 분리 |
| `robot_type_` | `std::shared_mutex` | 타입 읽기/쓰기 분리 |
| callbacks | `std::mutex` | 콜백 등록/호출 보호 |
| thread flags | `std::atomic<bool>` | 스레드 상태 플래그 |
| path update flags | `std::atomic<bool>` | 경로 업데이트 플래그 |

---

## 12. 파일 구조

```
src/slamnav-communication/
├── inc/
│   └── comm_zenoh.h              # 클래스 선언
└── src/
    ├── comm_zenoh.cpp            # 기본 구현 (세션, 스레드 관리, 콜백)
    └── zenoh_command/
        ├── comm_zenoh_control.cpp       # control_loop() 구현
        ├── comm_zenoh_localization.cpp  # localization_loop() 구현
        ├── comm_zenoh_map.cpp           # map_loop() 구현
        ├── comm_zenoh_move.cpp          # move_loop() 구현
        ├── comm_zenoh_moveStatus.cpp    # move_status_loop() 구현
        ├── comm_zenoh_status.cpp        # status_loop() 구현
        ├── SCHEMA_REFERENCE.md          # FlatBuffer 스키마 참조
        ├── TOPICS.md                    # 토픽 참조
        └── ARCHITECTURE.md              # 이 문서
```

---

## 13. Move 도메인 구현 (comm_zenoh_move.cpp)

### 13.1 개요

Move 도메인은 로봇의 이동 명령을 처리합니다.

| 통신 타입 | 토픽 | 설명 |
|----------|------|------|
| RPC (Queryable) | move/goal | 노드 ID/이름 기반 목표 이동 |
| RPC (Queryable) | move/target | 좌표 기반 목표 이동 |
| RPC (Queryable) | move/stop | 이동 정지 |
| RPC (Queryable) | move/pause | 이동 일시정지 |
| RPC (Queryable) | move/resume | 이동 재개 |
| RPC (Queryable) | move/xLinear | X축 직선 이동 |
| RPC (Queryable) | move/yLinear | Y축 직선 이동 (메카넘 전용) |
| RPC (Queryable) | move/circular | 원형 이동 |
| RPC (Queryable) | move/rotate | 제자리 회전 |
| Subscriber | move/jog | 실시간 조그 속도 명령 |
| Publisher | move/result | 이동 결과 발행 |

### 13.2 빌더 함수 패턴

익명 네임스페이스에서 FlatBuffer 응답을 생성하는 빌더 함수들을 정의합니다:

```cpp
namespace
{
  constexpr const char* MODULE_NAME = "ZENOH_MOVE";

  // 결과 빌더 (Publisher용)
  std::vector<uint8_t> build_result_move(const std::string& id,
                                         const std::string& goal_id,
                                         const std::string& goal_name,
                                         const std::string& method,
                                         int preset,
                                         const SLAMNAV::MovePose* goal_pose,
                                         float target,
                                         float speed,
                                         const std::string& direction,
                                         const std::string& result,
                                         const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(512);
    auto fb_result = SLAMNAV::CreateResultMove(fbb, ...);
    fbb.Finish(fb_result);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // RPC 응답 빌더 (Queryable용)
  std::vector<uint8_t> build_response_goal(const std::string& id,
                                           const std::string& goal_id,
                                           const std::string& goal_name,
                                           const std::string& method,
                                           int preset,
                                           const std::string& result,
                                           const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(512);
    auto resp = SLAMNAV::CreateResponseMoveGoal(fbb, ...);
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }
}
```

**빌더 함수 목록:**

| 함수명 | 용도 | FlatBuffer 타입 |
|--------|------|-----------------|
| `build_result_move` | 이동 결과 발행 | ResultMove |
| `build_profile_result` | 프로필 이동 결과 (헬퍼) | ResultMove |
| `build_response_goal` | goal RPC 응답 | ResponseMoveGoal |
| `build_response_target` | target RPC 응답 | ResponseMoveTarget |
| `build_response_stop` | stop RPC 응답 | ResponseMoveStop |
| `build_response_pause` | pause RPC 응답 | ResponseMovePause |
| `build_response_resume` | resume RPC 응답 | ResponseMoveResume |
| `build_response_xlinear` | xLinear RPC 응답 | ResponseMoveXLinear |
| `build_response_ylinear` | yLinear RPC 응답 | ResponseMoveYLinear |
| `build_response_circular` | circular RPC 응답 | ResponseMoveCircular |
| `build_response_rotate` | rotate RPC 응답 | ResponseMoveRotate |

### 13.3 move_loop() 구조

```cpp
void COMM_ZENOH::move_loop()
{
  log_info("move_loop started");

  // 1. robotType 설정 대기
  while (is_move_running_.load() && get_robot_type().empty())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // 2. 세션 유효성 확인
  if (!is_session_valid())
  {
    log_error("move_loop aborted: session invalid");
    return;
  }

  log_info("move_loop initialized with robotType: {}", get_robot_type());

  try
  {
    zenoh::Session& session = get_session();

    // 3. 토픽 생성
    std::string topic_goal     = make_topic("move/goal");
    std::string topic_target   = make_topic("move/target");
    // ...

    // 4. 결과 발행 람다
    auto send_result = [this, &topic_result](std::vector<uint8_t> data) {
      if (!is_session_valid()) return;
      zenoh::Session::GetOptions opts;
      opts.payload = zenoh::Bytes(std::move(data));
      get_session().get(zenoh::KeyExpr(topic_result), "", ...);
    };

    // 5. Subscriber 등록
    auto sub_jog = session.declare_subscriber(...);

    // 6. Queryable 등록
    auto q_goal = session.declare_queryable(...);
    auto q_target = session.declare_queryable(...);
    // ...

    // 7. 메인 루프
    while (is_move_running_.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log_info("move_loop ending, resources will be released");
  }
  catch (const zenoh::ZException& e)
  {
    log_error("move_loop Zenoh exception: {}", e.what());
  }
  catch (const std::exception& e)
  {
    log_error("move_loop exception: {}", e.what());
  }

  log_info("move_loop ended");
}
```

### 13.4 Queryable 핸들러 구현 패턴

모든 Queryable 핸들러는 다음 패턴을 따릅니다:

```cpp
auto q_goal = session.declare_queryable(
  zenoh::KeyExpr(topic_goal),
  [this](const zenoh::Query& query)
  {
    // 1. Payload 유효성 검사
    const auto& payload = query.get_payload();
    if (!payload.has_value())
    {
      auto resp = build_response_goal("", "", "", "", 0, "reject", "no payload");
      query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      return;
    }

    // 2. Slice 추출 및 FlatBuffer 파싱
    auto iter = payload->get().slice_iter();
    auto slice = iter.next();
    if (!slice.has_value())
    {
      auto resp = build_response_goal("", "", "", "", 0, "reject", "invalid slice, no data");
      query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      return;
    }
    auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveGoal>(slice->data);

    // 3. 요청 데이터 추출
    std::string id = req->id() ? req->id()->str() : "";
    std::string goal_id = req->goal_id() ? req->goal_id()->str() : "";
    // ...

    // 4. 모듈 의존성 획득 및 검증
    UNIMAP* unimap_ptr = get_unimap();
    LOCALIZATION* loc_ptr = get_localization();
    AUTOCONTROL* ctrl_ptr = get_autocontrol();
    MOBILE* mobile_ptr = get_mobile();

    if (!unimap_ptr || !loc_ptr || !ctrl_ptr || !mobile_ptr)
    {
      auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "module not ready");
      query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      return;
    }

    // 5. 비즈니스 로직 검증
    if (unimap_ptr->get_is_loaded() != MAP_LOADED)
    {
      auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "map not loaded");
      query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      return;
    }

    // 6. 명령 실행
    mobile_ptr->move(0, 0, 0);
    DATA_MOVE msg;
    // ... 메시지 구성

    // 7. 성공 응답
    auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "accept", "");
    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

    // 8. 비동기 작업 트리거 (필요시)
    Q_EMIT (ctrl_ptr->signal_move(msg));
  },
  zenoh::closures::none
);
```

### 13.5 응답 결과 값

| 결과 | 의미 |
|------|------|
| `"accept"` | 명령 수락 (처리 시작) |
| `"reject"` | 명령 거부 (오류) |
| `"success"` | 작업 완료 (결과 발행시) |
| `"fail"` | 작업 실패 (결과 발행시) |

### 13.6 Jog Subscriber 구현

실시간 조그 명령은 Subscriber로 처리합니다:

```cpp
auto sub_jog = session.declare_subscriber(
  zenoh::KeyExpr(topic_jog),
  [this](const zenoh::Sample& sample)
  {
    MOBILE* mobile_ptr = get_mobile();
    if (!mobile_ptr) return;

    const auto& payload = sample.get_payload();
    auto iter = payload.slice_iter();
    auto slice = iter.next();
    if (!slice.has_value()) return;

    auto jog = flatbuffers::GetRoot<SLAMNAV::MoveJog>(slice->data);
    if (!jog) return;

    double vx = static_cast<double>(jog->vx());
    double vy = static_cast<double>(jog->vy());
    double wz = static_cast<double>(jog->wz()) * D2R;

    mobile_ptr->slot_jog_update(Eigen::Vector3d(vx, vy, wz));
  },
  zenoh::closures::none
);
```

### 13.7 프로필 이동 비동기 결과 처리

프로필 이동(xLinear, yLinear, circular, rotate)은 `delayed_tasks`를 사용하여 비동기 결과를 발행합니다:

```cpp
// 즉시 응답
auto resp = build_response_xlinear(id, target, speed, "accept", "");
query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

// 이동 시작
ctrl_ptr->stop();
ctrl_ptr->set_is_moving(true);
mobile_ptr->move_linear_x(target, speed);

// 예상 완료 시간 후 결과 발행
double t = std::abs(target/(speed + 1e-06)) + 0.5;
delayed_tasks.schedule(std::chrono::milliseconds(static_cast<int>(t*1000)),
[this, send_result, id, target, speed]() {
  AUTOCONTROL* ctrl_ptr = get_autocontrol();
  MOBILE* mobile_ptr = get_mobile();
  if (!ctrl_ptr || !mobile_ptr)
  {
    send_result(build_profile_result(id, target, speed, "", "fail", "module not ready"));
    return;
  }
  ctrl_ptr->set_is_moving(false);
  send_result(build_profile_result(id, target, speed, "", "success", ""));
});
```

### 13.8 입력 값 검증 범위

| 명령 | 파라미터 | 최대값 |
|------|----------|--------|
| xLinear | target | 10.0 m |
| xLinear | speed | 1.5 m/s |
| yLinear | target | 10.0 m |
| yLinear | speed | 1.5 m/s |
| circular | target | 360.0° |
| circular | speed | 60.0°/s |
| rotate | target | 360.0° |
| rotate | speed | 60.0°/s |

---

## 14. 새 기능 구현 가이드

### 14.1 새 Pub/Sub 토픽 추가

1. **스키마 정의**: `schemas/amr/v1/slamnav_*.fbs`
2. **flatc 컴파일**: `flatc --cpp -o ...`
3. **루프 함수에서 발행/구독 구현**
4. **SCHEMA_REFERENCE.md 업데이트**

### 14.2 새 RPC 토픽 추가

1. **Request/Response 스키마 정의**
2. **Queryable 등록** (서버)
3. **Query 호출** (클라이언트)
4. **문서 업데이트**

### 14.3 새 스레드 추가

1. **헤더에 선언 추가**:
   - `is_xxx_running_` 플래그
   - `xxx_thread_` unique_ptr
   - `start_xxx_thread()`, `stop_xxx_thread()`, `xxx_loop()` 함수

2. **cpp에 구현 추가**:
   - 스레드 시작/정지 함수
   - 루프 함수

3. **start_all_thread/stop_all_thread에 추가**
