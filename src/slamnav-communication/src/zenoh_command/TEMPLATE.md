# zenoh_command 파일 작성 템플릿

## 1. 파일 구조

```cpp
/**
 * @file comm_zenoh_xxx.cpp
 * @brief [기능 설명]
 *
 * [패턴] Topic: {robotType}/[topic_suffix] ([주기]ms 주기)
 * 스키마: [schema_file].fbs ([root_type])
 */

#include "comm_zenoh.h"
#include "global_defines.h"
#include "flatbuffer/generated/[schema]_generated.h"

namespace
{
    constexpr const char* MODULE_NAME = "[MODULE_NAME]";  // 로그 prefix
    constexpr int [NAME]_INTERVAL_MS = [주기];            // 발행 주기 (Publisher인 경우)
}

// =============================================================================
// [기능] Publisher
// =============================================================================

static void publish_[name](COMM_ZENOH* zenoh)
{
    // 구현
}

// =============================================================================
// [기능] Loop Implementation
// =============================================================================

void COMM_ZENOH::[name]_loop()
{
    // 스레드 루프 구현
}
```

---

## 2. Include 규칙

```cpp
// 필수
#include "comm_zenoh.h"          // COMM_ZENOH 클래스, logger.h 포함 (→ spdlog, log_xxx 매크로 포함)
#include "global_defines.h"      // 전역 상수 (R2D, D2R, MAP_LOADED 등)

// FlatBuffer (필요한 스키마만)
#include "flatbuffer/generated/slamnav_xxx_generated.h"
```

**주의**:
- `spdlog/spdlog.h`는 별도로 include하지 않음 (comm_zenoh.h → logger.h에서 이미 포함)
- `<chrono>`는 스레드 루프에서 필요시 자동 포함됨

---

## 3. namespace 상수 정의

파일 상단에 익명 namespace로 상수 정의:

```cpp
namespace
{
    constexpr const char* MODULE_NAME = "MOVE_STATUS";  // 로그에 사용
    constexpr int MOVE_STATUS_INTERVAL_MS = 500;        // 발행 주기 (ms)
}
```

**MODULE_NAME 명명 규칙**:
- 대문자, 언더스코어 사용
- 예: `"STATUS"`, `"MOVE_STATUS"`, `"CONTROL"`, `"MAP"`

---

## 4. 로깅

### 4.1 로그 매크로 사용

`logger.h`에 정의된 매크로를 사용합니다. **spdlog 직접 호출 금지**.

```cpp
// Good - 매크로 사용
log_info("move_status_loop started");
log_info("initialized with robotType: {}", get_robot_type());
log_error("publish_move_status error: {}", e.what());

// Bad - spdlog 직접 사용 금지
spdlog::info("[MOVE_STATUS] ...");  // 사용하지 않음
spdlog::error("[ZENOH] ...");       // 사용하지 않음
```

### 4.2 로그 매크로 종류

| 매크로 | 용도 | 예시 |
|--------|------|------|
| `log_info()` | 일반 정보 | 스레드 시작/종료, 초기화 완료 |
| `log_debug()` | 디버깅용 | 상세 데이터 값 |
| `log_warn()` | 경고 | 복구 가능한 문제 |
| `log_error()` | 에러 | 예외 발생, 실패 |

### 4.3 로그 메시지 형식

```cpp
// 함수명_동작 형식
log_info("move_status_loop started");
log_info("move_status_loop ended");
log_info("move_status_loop ended (stopped before init)");
log_info("move_status_loop initialized with robotType: {}", get_robot_type());
log_error("publish_move_status error: {}", e.what());
```

---

## 5. 스레드 루프 패턴

### 5.1 기본 구조

```cpp
void COMM_ZENOH::[name]_loop()
{
    log_info("[name]_loop started");

    // 1. robotType 대기
    while (is_[name]_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 2. 종료 확인
    if (!is_[name]_running_.load())
    {
        log_info("[name]_loop ended (stopped before init)");
        return;
    }

    log_info("[name]_loop initialized with robotType: {}", get_robot_type());

    // 3. 타이밍 관리
    auto last_time = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds([INTERVAL]_MS);

    // 4. 메인 루프
    while (is_[name]_running_.load())
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_time >= interval)
        {
            publish_[name](this);
            last_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // CPU 사용량 최소화
    }

    log_info("[name]_loop ended");
}
```

### 5.2 robotType 대기 패턴

robotType이 설정되기 전에는 Topic을 발행하지 않습니다:

```cpp
while (is_[name]_running_.load() && get_robot_type().empty())
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
```

### 5.3 타이밍 관리

`std::chrono`를 사용한 정확한 주기 관리:

```cpp
auto last_time = std::chrono::steady_clock::now();
const auto interval = std::chrono::milliseconds(500);  // 500ms

while (running)
{
    auto now = std::chrono::steady_clock::now();
    if (now - last_time >= interval)
    {
        // 작업 수행
        last_time = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
```

---

## 6. static publish 함수 패턴

### 6.1 기본 구조

```cpp
static void publish_[name](COMM_ZENOH* zenoh)
{
    // 1. 연결 상태 확인
    if (!zenoh->get_is_connected())
    {
        return;
    }

    if (!zenoh->is_session_valid())
    {
        return;
    }

    // 2. 필수 모듈 확인 (getter 사용)
    AUTOCONTROL* ctrl = zenoh->get_autocontrol();
    MOBILE* mobile = zenoh->get_mobile();
    UNIMAP* unimap = zenoh->get_unimap();

    if (!ctrl || !mobile || !unimap)
    {
        return;
    }

    try
    {
        // 3. 데이터 수집
        // ...

        // 4. FlatBuffer 생성
        flatbuffers::FlatBufferBuilder fbb(1024);
        // ...
        fbb.Finish(root);

        // 5. Zenoh 발행
        const uint8_t* data = fbb.GetBufferPointer();
        size_t size = fbb.GetSize();
        std::string payload(reinterpret_cast<const char*>(data), size);

        std::string topic = zenoh->make_topic("[topic_suffix]");
        zenoh->get_session().put(zenoh::KeyExpr(topic), payload);
    }
    catch (const std::exception& e)
    {
        log_error("publish_[name] error: {}", e.what());
    }
}
```

### 6.2 모듈 접근 (static 함수에서)

static 함수에서는 `zenoh->get_xxx()` getter를 사용:

```cpp
// static 함수 내부
AUTOCONTROL* ctrl = zenoh->get_autocontrol();
MOBILE* mobile = zenoh->get_mobile();
UNIMAP* unimap = zenoh->get_unimap();
DOCKCONTROL* dctrl = zenoh->get_dockcontrol();
LOCALIZATION* loc = zenoh->get_localization();
```

---

## 7. Topic 정의

Topic 상수를 별도로 선언하지 않고 `make_topic()`에 직접 문자열 전달:

```cpp
// Good
std::string topic = zenoh->make_topic("moveStatus");
std::string topic = zenoh->make_topic("status");
std::string topic = zenoh->make_topic("move/goal");

// Bad (별도 상수 선언 금지)
constexpr const char* TOPIC_XXX = "xxx";  // 사용하지 않음
```

---

## 8. FlatBuffer 처리

### 8.1 Builder 생성

```cpp
flatbuffers::FlatBufferBuilder fbb(1024);  // 예상 크기
```

### 8.2 Table 생성

```cpp
auto node = SLAMNAV::CreateMoveStatusNode(fbb,
    fbb.CreateString(id.toStdString()),
    fbb.CreateString(name.toStdString()),
    fbb.CreateString(""),
    static_cast<float>(x),
    static_cast<float>(y),
    static_cast<float>(theta)
);
```

### 8.3 Struct 생성 (inline)

```cpp
SLAMNAV::MoveStatusPose pose(
    static_cast<float>(x),
    static_cast<float>(y),
    static_cast<float>(z),
    static_cast<float>(theta)
);

// Root에서 포인터로 전달
auto root = SLAMNAV::CreateXxx(fbb, ..., &pose, ...);
```

### 8.4 Root 완료 및 발행

```cpp
fbb.Finish(root);

const uint8_t* data = fbb.GetBufferPointer();
size_t size = fbb.GetSize();
std::string payload(reinterpret_cast<const char*>(data), size);

std::string topic = zenoh->make_topic("moveStatus");
zenoh->get_session().put(zenoh::KeyExpr(topic), payload);
```

---

## 9. 에러 처리

```cpp
try
{
    // ...
}
catch (const std::exception& e)
{
    log_error("publish_[name] error: {}", e.what());
}
```

---

## 10. 예제: comm_zenoh_moveStatus.cpp

```cpp
#include "comm_zenoh.h"
#include "global_defines.h"
#include "flatbuffer/generated/slamnav_status_generated.h"

namespace
{
    constexpr const char* MODULE_NAME = "MOVE_STATUS";
    constexpr int MOVE_STATUS_INTERVAL_MS = 500;  // 500ms
}

void COMM_ZENOH::move_status_loop()
{
    log_info("move_status_loop started");

    while (is_move_status_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_move_status_running_.load())
    {
        log_info("move_status_loop ended (stopped before init)");
        return;
    }

    log_info("move_status_loop initialized with robotType: {}", get_robot_type());

    auto last_time = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(MOVE_STATUS_INTERVAL_MS);

    while (is_move_status_running_.load())
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_time >= interval)
        {
            publish_move_status(this);
            last_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    log_info("move_status_loop ended");
}


static void publish_move_status(COMM_ZENOH* zenoh)
{
    if (!zenoh->get_is_connected())
    {
        return;
    }

    if (!zenoh->is_session_valid())
    {
        return;
    }

    AUTOCONTROL* ctrl = zenoh->get_autocontrol();
    MOBILE* mobile = zenoh->get_mobile();
    UNIMAP* unimap = zenoh->get_unimap();
    DOCKCONTROL* dctrl = zenoh->get_dockcontrol();
    LOCALIZATION* loc = zenoh->get_localization();

    if (!ctrl || !mobile || !unimap || !dctrl || !loc)
    {
        return;
    }

    try
    {
        const MOBILE_POSE mo = mobile->get_pose();
        const Eigen::Vector3d cur_xi = TF_to_se2(loc->get_cur_tf());

        QString cur_node_id = ctrl->get_cur_node_id();
        QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;

        // cur node
        QString cur_node_name = "";
        if (unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
        {
            if (NODE* node = unimap->get_node_by_id(cur_node_id))
            {
                cur_node_name = node->name;
            }
        }

        // goal node
        QString goal_node_name = "";
        Eigen::Vector3d goal_xi(0, 0, 0);
        if (unimap->get_is_loaded() == MAP_LOADED && !goal_node_id.isEmpty())
        {
            if (NODE* node = unimap->get_node_by_id(goal_node_id))
            {
                goal_node_name = node->name;
                goal_xi = TF_to_se2(node->tf);
            }
        }

        flatbuffers::FlatBufferBuilder fbb(1024);

        // MoveStatusMoveState (table)
        auto move_state = SLAMNAV::CreateMoveStatusMoveState(fbb,
            fbb.CreateString(ctrl->get_auto_state().toStdString()),
            fbb.CreateString("stop"),
            fbb.CreateString("none"),
            fbb.CreateString(ctrl->get_obs_condition().toStdString()),
            fbb.CreateString(ctrl->get_multi_reqest_state().toStdString())
        );

        // MoveStatusPose (struct)
        SLAMNAV::MoveStatusPose pose(
            static_cast<float>(cur_xi[0]),
            static_cast<float>(cur_xi[1]),
            0.0f,
            static_cast<float>(cur_xi[2] * R2D)
        );

        // MoveStatusVel (struct)
        SLAMNAV::MoveStatusVel vel(
            static_cast<float>(mo.vel[0]),
            static_cast<float>(mo.vel[1]),
            static_cast<float>(mo.vel[2] * R2D)
        );

        // MoveStatusNode (cur_node) (table)
        auto cur_node = SLAMNAV::CreateMoveStatusNode(fbb,
            fbb.CreateString(cur_node_id.toStdString()),
            fbb.CreateString(cur_node_name.toStdString()),
            fbb.CreateString(""),
            static_cast<float>(cur_xi[0]),
            static_cast<float>(cur_xi[1]),
            static_cast<float>(cur_xi[2] * R2D)
        );

        // MoveStatusNode (goal_node) (table)
        auto goal_node = SLAMNAV::CreateMoveStatusNode(fbb,
            fbb.CreateString(goal_node_id.toStdString()),
            fbb.CreateString(goal_node_name.toStdString()),
            fbb.CreateString(ctrl->get_cur_move_state().toStdString()),
            static_cast<float>(goal_xi[0]),
            static_cast<float>(goal_xi[1]),
            static_cast<float>(goal_xi[2] * R2D)
        );

        // Move_Status (root)
        auto ms = SLAMNAV::CreateMove_Status(fbb,
            cur_node,
            goal_node,
            move_state,
            &pose,
            &vel
        );
        fbb.Finish(ms);

        // Zenoh로 발행
        const uint8_t* data = fbb.GetBufferPointer();
        size_t size = fbb.GetSize();
        std::string payload(reinterpret_cast<const char*>(data), size);

        std::string topic = zenoh->make_topic("moveStatus");
        zenoh->get_session().put(zenoh::KeyExpr(topic), payload);
    }
    catch (const std::exception& e)
    {
        log_error("publish_move_status error: {}", e.what());
    }
}
```

---

## 11. 체크리스트

새 파일 작성 시 확인:

### 필수 항목
- [ ] `#include "comm_zenoh.h"` 포함
- [ ] `#include "global_defines.h"` 포함
- [ ] 필요한 FlatBuffer generated 헤더 포함
- [ ] spdlog 별도 include 하지 않음

### namespace 상수
- [ ] `MODULE_NAME` 정의 (대문자, 로그용)
- [ ] 발행 주기 상수 정의 (Publisher인 경우)

### 로깅
- [ ] `log_info`, `log_error` 등 매크로 사용
- [ ] `spdlog::xxx()` 직접 호출하지 않음
- [ ] 로그 메시지: `"함수명 동작"` 형식

### 스레드 루프
- [ ] robotType 대기 패턴 적용
- [ ] 종료 전 running 플래그 확인
- [ ] 타이밍 관리 (`std::chrono` 사용)
- [ ] 메인 루프 sleep (10ms)

### publish 함수
- [ ] `static void publish_xxx(COMM_ZENOH* zenoh)` 형식
- [ ] `zenoh->get_is_connected()` 확인
- [ ] `zenoh->is_session_valid()` 확인
- [ ] 모듈 getter 사용 (`zenoh->get_xxx()`)
- [ ] 모듈 null 체크
- [ ] try-catch로 예외 처리

### Topic
- [ ] `zenoh->make_topic("xxx")` 직접 사용
- [ ] Topic 상수 별도 선언 안함
