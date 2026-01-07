# AUTOCONTROL 모듈 가이드

## 개요

AUTOCONTROL은 로봇의 자율 주행 및 경로 추종 제어를 담당하는 핵심 모듈입니다.

**주요 기능:**
- 전역/지역 경로 계획 (Global/Local Path Planning)
- 실시간 경로 추종 제어
- 장애물 감지 및 회피
- 멀티 로봇 협조 제어 지원
- FSM 기반 이동 상태 관리

**설계 패턴:**
- 싱글톤 패턴 (`AUTOCONTROL::instance()`)
- Qt QObject 기반 (Signal/Slot 통신)
- 멀티스레드 아키텍처

**파일 위치:**
- 헤더: `src/slamnav-navigation/inc/autocontrol.h`
- 구현: `src/slamnav-navigation/src/autocontrol.cpp`

---

## 아키텍처

### 스레드 구조

AUTOCONTROL은 3개의 병렬 스레드로 동작합니다:

```
┌─────────────────────────────────────────────────────────┐
│                    AUTOCONTROL                          │
├─────────────────┬─────────────────┬─────────────────────┤
│  control_loop   │    obs_loop     │     node_loop       │
│  (경로 추종)     │  (장애물 감지)   │   (노드 추적)        │
│  ~50Hz          │    ~50Hz        │      ~50Hz          │
└─────────────────┴─────────────────┴─────────────────────┘
```

| 스레드 | 역할 | 주기 |
|--------|------|------|
| `control_loop` | FSM 상태 머신 실행, 경로 추종 제어, 속도 명령 생성 | 50Hz (20ms) |
| `obs_loop` | 장애물 감지, 충돌 예측, 안전 속도 계산 | 50Hz (20ms) |
| `node_loop` | 현재 노드 추적, 멀티 로봇 step 관리 | 50Hz (20ms) |

### 스레드 안전성

```cpp
std::recursive_mutex mtx;          // 메인 경로/상태 락
std::mutex path_mtx;               // 멀티 로봇 경로 락
std::mutex path_st_node_mtx;       // 시작 노드 락

std::atomic<bool> control_flag;    // 제어 루프 활성화 플래그
std::atomic<bool> obs_flag;        // 장애물 루프 플래그
std::atomic<bool> node_flag;       // 노드 루프 플래그
std::atomic<int> fsm_state;        // FSM 상태
```

### 모듈 의존성

```
                    ┌──────────────┐
                    │  AUTOCONTROL │
                    └──────┬───────┘
                           │
       ┌───────────────────┼───────────────────┐
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   MOBILE    │    │LOCALIZATION │    │   OBSMAP    │
│ (모터 제어)  │    │ (위치 추정)  │    │ (장애물 맵)  │
└─────────────┘    └─────────────┘    └─────────────┘
       │                   │                   │
       └───────────────────┼───────────────────┘
                           │
       ┌───────────────────┼───────────────────┐
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   UNIMAP    │    │   POLICY    │    │   CONFIG    │
│ (토폴로지맵) │    │ (주행 정책)  │    │   (설정)    │
└─────────────┘    └─────────────┘    └─────────────┘
```

---

## 상태 머신 (FSM)

### AUTO_FSM_STATE - 메인 상태

```
┌─────────────────┐
│ AUTO_FSM_FIRST  │  초기 헤딩 정렬
│     _ALIGN      │  (경로 시작점 방향으로 회전)
└────────┬────────┘
         │ 헤딩 정렬 완료
         ▼
┌─────────────────┐
│ AUTO_FSM_DRIVING│  경로 추종 주행
│                 │  (Stanley/PP 제어)
└────────┬────────┘
         │ 목표 근접
         ▼
┌─────────────────┐
│ AUTO_FSM_FINAL  │  최종 위치/헤딩 정렬
│     _ALIGN      │
└────────┬────────┘
         │ 목표 도달
         ▼
┌─────────────────┐
│ AUTO_FSM_COMPLETE│  이동 완료
│                 │
└─────────────────┘

[장애물 감지 시]
         │
         ▼
┌─────────────────┐
│  AUTO_FSM_OBS   │  장애물 대응
│                 │  (대기/회피)
└─────────────────┘
```

| 상태 | 값 | 설명 |
|------|-----|------|
| `AUTO_FSM_FIRST_ALIGN` | 0 | 초기 헤딩 정렬 |
| `AUTO_FSM_DRIVING` | 1 | 경로 추종 주행 |
| `AUTO_FSM_FINAL_ALIGN` | 2 | 최종 정렬 |
| `AUTO_FSM_OBS` | 3 | 장애물 대응 |
| `AUTO_FSM_COMPLETE` | 4 | 이동 완료 |
| `AUTO_FSM_DOCKING` | 5 | 도킹 모드 |
| `AUTO_FSM_PAUSE` | 6 | 일시 정지 |

### StateObsCondition - 장애물 상태

| 상태 | 설명 |
|------|------|
| `NONE` | 장애물 없음 |
| `FAR` | 먼 거리 장애물 감지 |
| `NEAR` | 근거리 장애물 감지 (감속/정지) |
| `VIR` | 가상 장애물 (멀티 로봇 충돌 방지) |

### StateMultiReq - 멀티 로봇 요청 상태

| 상태 | 설명 |
|------|------|
| `NONE` | 요청 없음 |
| `REQ_PATH` | 경로 요청 중 |
| `RECV_PATH` | 경로 수신 완료 |

### StateCurGoal - 목표 상태

| 상태 | 설명 |
|------|------|
| `NONE` | 목표 없음 |
| `MOVE` | 이동 중 |
| `COMPLETE` | 목표 도달 완료 |
| `FAIL` | 이동 실패 |
| `OBSTACLE` | 장애물로 인한 중단 |
| `CANCEL` | 취소됨 |

---

## 주요 API

### 이동 명령 (Slots)

```cpp
// 단일 로봇 이동 (목표 좌표)
void slot_move(DATA_MOVE msg);

// 후진 이동
void slot_move_backward(DATA_MOVE msg);

// 멀티 로봇 이동 (서버에서 받은 경로)
void slot_move_multi();

// 프로파일 이동 (직선/원호)
void slot_profile_move(DATA_MOVE msg);

// 긴급 정지
void stop();
```

### 경로 설정

```cpp
// 멀티 로봇 경로 설정
void set_path(
    const std::vector<QString>& _global_node_path,  // 노드 경로
    int _global_preset,                              // preset 번호
    long long _global_path_time                      // 타임스탬프
);
```

### 상태 조회 (Getters)

```cpp
int get_fsm_state();                    // FSM 상태
bool get_is_moving();                   // 이동 중 여부
bool get_is_pause();                    // 일시 정지 여부
QString get_auto_state();               // 종합 상태 ("stop"/"move"/"pause"/"error"/"vir")
QString get_cur_move_state();           // 이동 상태 ("none"/"move"/"complete"/"fail"/"obstacle"/"cancel")
QString get_obs_condition();            // 장애물 상태 ("none"/"near"/"far"/"vir")
QString get_cur_node_id();              // 현재 노드 ID
QString get_multi_reqest_state();       // 멀티 요청 상태 ("none"/"req_path"/"recv_path")
```

### 경로 조회

```cpp
PATH get_cur_global_path();             // 전역 경로
PATH get_cur_local_path();              // 지역 경로 (고해상도)
double get_obs_dist();                  // 장애물까지 거리
double get_cur_deadzone();              // 현재 안전 거리
```

### 성능 모니터링

```cpp
double get_process_time_control();      // 제어 루프 처리 시간
double get_process_time_node();         // 노드 루프 처리 시간
double get_process_time_obs();          // 장애물 루프 처리 시간
```

### 모듈 설정

```cpp
void set_config_module(CONFIG* _config);
void set_logger_module(LOGGER* _logger);
void set_mobile_module(MOBILE* _mobile);
void set_unimap_module(UNIMAP* _unimap);
void set_obsmap_module(OBSMAP* _obsmap);
void set_policy_module(POLICY* _policy);
void set_localization_module(LOCALIZATION* _localization);
```

---

## 경로 계획 함수

### 전역 경로 계획

```cpp
// 목표 좌표 기반 (단일 로봇)
PATH calc_global_path(Eigen::Matrix4d goal);

// 노드 경로 기반 (멀티 로봇)
PATH calc_global_path(std::vector<QString> node_path, bool add_cur_tf);
```

### 지역 경로 계획

```cpp
// 고해상도 지역 경로 계산
PATH calc_local_path(PATH& global_path);

// 회피 경로 계산 (Hybrid A*)
PATH calc_avoid_path(PATH& global_path);
```

### 경로 처리

```cpp
// 경로 업샘플링 (해상도 증가)
std::vector<Eigen::Vector3d> path_resampling(const std::vector<Eigen::Vector3d>& src, double step);
std::vector<Eigen::Matrix4d> path_resampling(const std::vector<Eigen::Matrix4d>& src, double step);

// CCMA 스무딩 (곡률 제한 이동 평균)
std::vector<Eigen::Vector3d> path_ccma(const std::vector<Eigen::Vector3d>& src);

// 기준 속도 계산
void calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_v, double st_v, double step);

// 속도 스무딩
std::vector<double> smoothing_v(const std::vector<double>& src, double path_step);
```

### 멀티 로봇 경로 분할

```cpp
// 대칭 경로 분할 (1-2-3-2-1 → [1-2-3], [3-2-1])
std::vector<std::vector<QString>> symmetric_cut(std::vector<QString> node_path);

// 루프 경로 분할 (1-2-3-1-2 → [1-2-3], [1-2])
std::vector<std::vector<QString>> loop_cut(std::vector<QString> node_path);
```

---

## 제어 알고리즘

### 주행 방식 (DriveMethod)

| 방식 | 설명 | 적용 로봇 |
|------|------|----------|
| `METHOD_PP` | Pure Pursuit | 차동 구동 (S100-A/B 등) |
| `METHOD_HPP` | Hybrid Pure Pursuit | 메카넘 휠 (MECANUM 시리즈) |
| `METHOD_SIDE` | 측면 이동 | 홀로노믹 로봇 |

### 제어 로직 개요

**FIRST_ALIGN 상태:**
- PID 피벗 제어로 초기 헤딩 정렬
- 경로 시작 방향으로 회전

**DRIVING 상태:**
- Stanley/Pure Pursuit 제어
- Cross-track error 보정
- 적응형 속도 제어

**FINAL_ALIGN 상태:**
- 최종 위치/헤딩 미세 조정
- 저속 접근

---

## 설정 파라미터

### preset JSON 파일 위치

```
/data/slamnav2/[ROBOT_TYPE]/config/preset_[N].json

예시:
/data/slamnav2/S100-A/config/preset_0.json
/data/slamnav2/MECANUM-Q150/config/preset_0.json
```

### CTRL_PARAM 구조체

```cpp
struct CTRL_PARAM
{
    // 속도 제한
    double LIMIT_V;        // 최대 선속도 (m/s)
    double LIMIT_W;        // 최대 각속도 (deg/s)
    double LIMIT_V_ACC;    // 선가속도 제한 (m/s²)
    double LIMIT_V_DCC;    // 선감속도 제한 (m/s²)
    double LIMIT_W_ACC;    // 각가속도 제한 (deg/s²)
    double LIMIT_PIVOT_W;  // 피벗 회전 각속도 제한 (deg/s)

    // 시작/종료 속도
    double ST_V;           // 시작 속도 (m/s)
    double ED_V;           // 종료 속도 (m/s)

    // 제어 게인
    double DRIVE_T;        // 시간 파라미터
    double DRIVE_H;        // 헤딩 제어 게인
    double DRIVE_A;        // PD 비례 게인
    double DRIVE_B;        // PD 미분 게인
    double DRIVE_L;        // Look-ahead 거리 (m)
    double DRIVE_K;        // Cross-track error 게인 (Stanley)
    double DRIVE_EPS;      // 수렴 허용 오차
};
```

### 예시 설정 (S100-A)

```json
{
    "LIMIT_V": "1.3",
    "LIMIT_W": "50.0",
    "LIMIT_V_ACC": "0.3",
    "LIMIT_V_DCC": "0.5",
    "LIMIT_W_ACC": "180.0",
    "LIMIT_PIVOT_W": "60.0",
    "ST_V": "0.3",
    "ED_V": "0.3",
    "DRIVE_T": "0.0",
    "DRIVE_H": "4.0",
    "DRIVE_A": "0.9",
    "DRIVE_B": "0.03",
    "DRIVE_L": "0.4",
    "DRIVE_K": "1.0",
    "DRIVE_EPS": "0.3"
}
```

### 내부 상수 (AUTOCONTROL_INFO)

```cpp
static constexpr int control_loop_cnt = 20;               // 루프 카운터 (주기 계산용)
static constexpr double local_path_calc_dt = 0.1;         // 지역 경로 재계산 주기 (s)
static constexpr double local_path_step = 0.01;           // 지역 경로 해상도 (m)
static constexpr double global_path_step = 0.1;           // 전역 경로 해상도 (m)
static constexpr double obstacle_near_check_dist = 2.5;   // 근거리 장애물 판정 거리 (m)
static constexpr double dynamic_deadzone_release_rate = 0.05;  // 데드존 해제율
static constexpr double obs_decel_recover_rate = 0.1;     // 감속 회복율
```

---

## 외부 모듈 연동

### MOBILE - 모터 제어

```cpp
mobile->move(vx, vy, wz);           // 속도 명령 전송
mobile->moveQD(vx, vy, wz, type);   // 타입별 이동 (메카넘)
mobile->get_cur_pdu_state();        // 모터 상태 확인 ("good"/에러)
mobile->get_control_input();        // 현재 속도 입력값
mobile->get_pose();                 // 현재 위치/속도
mobile->set_is_auto_move(bool);     // 자율 주행 플래그 설정
```

### LOCALIZATION - 위치 추정

```cpp
loc->get_cur_tf();                  // 현재 변환 행렬 (4x4)
loc->get_cur_loc_state();           // 위치 추정 상태 ("good"/에러)
```

### OBSMAP - 장애물 감지

```cpp
obsmap->is_path_collision(traj, ...);  // 경로 충돌 검사
obsmap->is_tf_collision(tf, ...);      // 위치 충돌 검사
obsmap->calc_path(st_tf, ed_tf);       // 회피 경로 계산 (Hybrid A*)
obsmap->get_obs_pts();                 // 장애물 포인트
obsmap->get_vir_pts();                 // 가상 장애물 포인트 (멀티 로봇)
obsmap->clear();                       // 장애물 맵 초기화
```

### UNIMAP - 토폴로지 맵

```cpp
unimap->get_node_by_id(node_id);       // 노드 정보 조회
unimap->get_edge_nodes(cur_pos);       // 현재 위치 인접 엣지
unimap->get_links();                   // 모든 링크 조회
unimap->get_node_id_edge(pos);         // 위치에서 가장 가까운 노드
```

### POLICY - 주행 정책

```cpp
policy->drive_policy(path);            // 주행 정책 적용
policy->speed_policy(path, ref_v);     // 속도 정책 적용
```

---

## Signal/Slot 인터페이스

### Signals (출력)

```cpp
// 이동 명령 시그널
void signal_move(DATA_MOVE msg);
void signal_move_backward(DATA_MOVE msg);
void signal_move_multi();

// 응답 시그널
void signal_move_response(DATA_MOVE msg);    // 이동 완료/실패 응답

// 경로 업데이트 시그널
void signal_global_path_updated();           // 전역 경로 변경됨
void signal_local_path_updated();            // 지역 경로 변경됨
```

### Slots (입력)

```cpp
void slot_move(DATA_MOVE msg);               // 이동 명령 수신
void slot_move_backward(DATA_MOVE msg);      // 후진 명령 수신
void slot_move_multi();                      // 멀티 로봇 이동 시작
void slot_profile_move(DATA_MOVE msg);       // 프로파일 이동
void stop();                                 // 긴급 정지
```

### 통신 모듈 연동

**comm_msa.cpp / comm_fms.cpp에서 호출:**

```cpp
// 이동 명령 전송
emit autocontrol->signal_move(msg);

// 멀티 로봇 경로 설정
autocontrol->set_path(node_path, preset, timestamp);
emit autocontrol->signal_move_multi();

// 응답 수신 연결
connect(autocontrol, &AUTOCONTROL::signal_move_response,
        this, &COMM_MSA::handle_move_response);
```

---

## 사용 예시

### 초기화

```cpp
// 싱글톤 인스턴스 획득
AUTOCONTROL* auto_ctrl = AUTOCONTROL::instance();

// 모듈 설정
auto_ctrl->set_config_module(config);
auto_ctrl->set_logger_module(logger);
auto_ctrl->set_mobile_module(mobile);
auto_ctrl->set_unimap_module(unimap);
auto_ctrl->set_obsmap_module(obsmap);
auto_ctrl->set_policy_module(policy);
auto_ctrl->set_localization_module(loc);

// 초기화
auto_ctrl->init();
```

### 단일 로봇 이동

```cpp
DATA_MOVE msg;
msg.command = "goal";
msg.tgt_pose_vec = {x, y, z, yaw};  // 목표 좌표
msg.preset = 0;                      // preset_0.json 사용
msg.drive_method = "pp";             // Pure Pursuit

emit auto_ctrl->signal_move(msg);
```

### 멀티 로봇 경로 설정

```cpp
std::vector<QString> node_path = {"N001", "N002", "N003", "N004"};
int preset = 0;
long long timestamp = getCurrentTimestamp();

auto_ctrl->set_path(node_path, preset, timestamp);
emit auto_ctrl->signal_move_multi();
```

### 상태 모니터링

```cpp
// FSM 상태 확인
int state = auto_ctrl->get_fsm_state();
if (state == AUTO_FSM_DRIVING) {
    // 주행 중
}

// 종합 상태 확인
QString auto_state = auto_ctrl->get_auto_state();
// "stop", "move", "pause", "error", "vir"

// 장애물 상태 확인
QString obs = auto_ctrl->get_obs_condition();
// "none", "near", "far", "vir"

// 이동 결과 확인
QString move_state = auto_ctrl->get_cur_move_state();
// "none", "move", "complete", "fail", "obstacle", "cancel"
```

### 긴급 정지

```cpp
auto_ctrl->stop();
```

---

## 주의사항 및 디버깅

### 일반적인 문제

1. **이동이 시작되지 않음**
   - `loc->get_cur_loc_state()` 확인 (위치 추정 상태)
   - `mobile->get_cur_pdu_state()` 확인 (모터 상태)
   - preset 파일 존재 여부 확인

2. **경로 추종 오차가 큼**
   - `DRIVE_K`, `DRIVE_L` 파라미터 조정
   - `local_path_step` 확인 (해상도)

3. **장애물 감지 과민/둔감**
   - `obstacle_near_check_dist` 조정
   - `config->get_obs_safe_margin_*()` 확인

4. **멀티 로봇 충돌**
   - 가상 장애물(`VIR`) 상태 확인
   - `obsmap->get_vir_pts()` 확인

### 로그 확인 포인트

```cpp
// 제어 루프 처리 시간
double ctrl_time = auto_ctrl->get_process_time_control();

// 장애물 루프 처리 시간
double obs_time = auto_ctrl->get_process_time_obs();

// 현재 상태
spdlog::info("FSM: {}, Auto: {}, Move: {}",
    auto_ctrl->get_fsm_state(),
    auto_ctrl->get_auto_state().toStdString(),
    auto_ctrl->get_cur_move_state().toStdString());
```

### 성능 지표

| 지표 | 정상 범위 | 확인 방법 |
|------|----------|----------|
| control_loop 주기 | < 25ms | `get_process_time_control()` |
| obs_loop 주기 | < 25ms | `get_process_time_obs()` |
| node_loop 주기 | < 25ms | `get_process_time_node()` |

---

## 지원 로봇 타입

| 로봇 타입 | 구동 방식 | 기본 DriveMethod |
|----------|----------|-----------------|
| S100-A / S100-B | 차동 구동 | PP |
| S100-A-3D / S100-B-3D | 차동 구동 | PP |
| MECANUM-Q150 | 메카넘 휠 | HPP |
| MECANUM-SEC_CORE | 메카넘 휠 | HPP |
| MECANUM-VALEO | 메카넘 휠 | HPP |
| QD | 4륜 구동 | HPP |
| DD-SEC_EE | 차동 구동 | PP |
| D400 | 차동 구동 | PP |
| SEM / SEM-D1000 | 차동 구동 | PP |
| SDC | 차동 구동 | PP |
| SRA | 차동 구동 | PP |
