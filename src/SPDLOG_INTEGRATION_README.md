# AMR 시스템을 위한 spdlog 통합 가이드

## 개요

본 문서는 AMR(자율주행로봇) 시스템에 spdlog 라이브러리를 통합하여 고성능, 구조화된 로깅 시스템을 구축하는 방법을 설명합니다.

## 주요 기능

### 1. 모듈별 로거 분리
- **Communication**: 통신 관련 로그
- **Navigation**: 경로 계획 및 내비게이션 로그
- **Safety**: 안전 시스템 로그
- **MissionControl**: 미션 제어 로그
- **System**: 시스템 전체 로그

### 2. 성능 최적화
- **비동기 로깅**: 메인 스레드 블로킹 방지
- **컴파일 타임 최적화**: SPDLOG_ACTIVE_LEVEL 매크로 사용
- **Rotating File Sink**: 자동 로그 파일 관리

### 3. 구조화된 로깅
- **JSON 형식**: 기계가 읽기 쉬운 구조화된 로그
- **이벤트 기반**: 이벤트 타입별 분류
- **페이로드 지원**: 복잡한 데이터 구조 지원

## 설치 및 설정

### 1. spdlog 설치
```bash
cd ~
git clone https://github.com/gabime/spdlog.git
cd spdlog
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

### 2. 프로젝트 설정
`SLAMNAV2.pro` 파일에 다음 내용이 자동으로 추가됩니다:
```qmake
# spdlog
INCLUDEPATH += $$HOME/spdlog/include/
LIBS += -L$$HOME/spdlog/build/
LIBS += -lspdlog

# spdlog 컴파일 타임 최적화
DEFINES += SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO
```

## 사용법

### 1. 기본 로거 사용
```cpp
#include "logger.h"

// 로거 초기화
LOGGER* logger = LOGGER::instance();
logger->init();

// 모듈별 로거 사용
if(auto comm_logger = spdlog::get("Communication"))
{
    comm_logger->info("Communication module initialized");
    comm_logger->debug("Debug message");
    comm_logger->warn("Warning message");
    comm_logger->error("Error message");
}
```

### 2. 구조화된 로깅
```cpp
// JSON 페이로드 생성
QJsonObject payload;
payload["robot_id"] = "AMR-001";
payload["battery_level"] = 85.5;
payload["current_position"] = QJsonObject{
    {"x", 10.5},
    {"y", 20.3},
    {"theta", 1.57}
};

// 구조화된 이벤트 로깅
logger->log_structured_event("RobotStatus", "Robot is moving to target", payload, spdlog::level::info);
```

### 3. 성능 최적화된 로깅
```cpp
// 컴파일 타임에 제거되는 로그 (릴리즈 빌드에서)
SPDLOG_TRACE("This will be removed in release");
SPDLOG_DEBUG("This will be removed in release");

// 항상 유지되는 로그
SPDLOG_INFO("This will be kept");
SPDLOG_WARN("This will be kept");
SPDLOG_ERROR("This will be kept");
```

## 로그 파일 구조

### 1. 파일 위치
- **콘솔 출력**: 실시간 모니터링용
- **comm.log**: 통신 모듈 로그 (10MB, 5개 파일 유지)
- **navigation.log**: 내비게이션 모듈 로그
- **safety.log**: 안전 시스템 로그
- **mission.log**: 미션 제어 로그
- **system.log**: 시스템 전체 로그

### 2. 로그 레벨
- **trace**: 가장 상세한 디버깅 정보
- **debug**: 개발용 디버깅 정보
- **info**: 일반적인 정보
- **warn**: 경고 메시지
- **error**: 오류 메시지
- **critical**: 치명적 오류

## 성능 최적화 설정

### 1. 컴파일 타임 최적화
```cpp
// 프로덕션 빌드에서 디버그 로그 제거
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
```

### 2. 비동기 로깅
- 모든 로거가 자동으로 비동기 처리
- 메인 스레드 블로킹 방지
- 고주파 로깅 지원

### 3. 플러시 정책
```cpp
// 오류 발생 시 즉시 플러시
spdlog::flush_on(spdlog::level::err);

// 1초마다 주기적 플러시
spdlog::flush_every(std::chrono::seconds(1));
```

## 모니터링 및 분석

### 1. 실시간 모니터링
```bash
# 실시간 로그 확인
tail -f logs/comm.log

# 특정 이벤트 필터링
grep "RobotStatus" logs/comm.log
```

### 2. JSON 로그 분석
```bash
# JSON 로그 파싱
cat logs/comm.log | jq '.event_type'

# 특정 로봇 ID 필터링
cat logs/comm.log | jq 'select(.payload.robot_id == "AMR-001")'
```

## 문제 해결

### 1. 로거 초기화 실패
```cpp
try {
    logger->init();
} catch(const spdlog::spdlog_ex& ex) {
    printf("SPDLOG init failed: %s\n", ex.what());
}
```

### 2. 로그 파일 권한 문제
```bash
# 로그 디렉토리 권한 확인
ls -la logs/
chmod 755 logs/
```

### 3. 메모리 사용량 모니터링
```bash
# 로그 파일 크기 확인
du -sh logs/*.log

# 로그 파일 개수 확인
ls -la logs/*.log | wc -l
```

## 모범 사례

### 1. 로그 메시지 작성
```cpp
// 좋은 예
comm_logger->info("Connection established to server {}:{}", host, port);

// 나쁜 예
comm_logger->info("Connected");
```

### 2. 구조화된 데이터 사용
```cpp
// 좋은 예
QJsonObject payload;
payload["server"] = host;
payload["port"] = port;
payload["status"] = "connected";
logger->log_structured_event("Connection", "Server connection established", payload);

// 나쁜 예
comm_logger->info("Connected to {}:{}", host, port);
```

### 3. 로그 레벨 적절히 사용
```cpp
// trace: 매우 상세한 디버깅
SPDLOG_TRACE("Function entry: {}", __FUNCTION__);

// debug: 개발용 정보
comm_logger->debug("Processing message: {}", message_id);

// info: 일반적인 정보
comm_logger->info("User {} logged in", user_id);

// warn: 경고 상황
comm_logger->warn("Battery level low: {}%", battery_level);

// error: 오류 상황
comm_logger->error("Failed to connect to server: {}", error_code);
```

## 결론

spdlog 통합을 통해 AMR 시스템은 다음과 같은 이점을 얻을 수 있습니다:

1. **고성능**: 비동기 로깅으로 실시간 성능 보장
2. **구조화**: JSON 기반 로그로 쉬운 분석
3. **모듈화**: 서브시스템별 로그 분리
4. **확장성**: Fleet 단위 로그 집계 준비
5. **안정성**: 견고한 플러시 정책으로 데이터 손실 방지

이러한 로깅 시스템은 AMR의 안정적인 운영과 효율적인 문제 해결에 크게 기여할 것입니다.
