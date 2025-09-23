# spdlog 로그 레벨 설정 가이드

## 개요
config.cpp에서 spdlog의 로그 레벨을 관리할 수 있도록 기능을 추가했습니다.

## 설정 방법

### 1. config.json에서 로그 레벨 설정
```json
{
    "logging": {
        "LOG_LEVEL": "info",
        "LOG_ENABLE_LIDAR_DEBUG": "false"
    }
}
```

### 2. 사용 가능한 로그 레벨
- `"trace"`: 모든 로그 출력
- `"debug"`: 디버그 레벨 이상 출력
- `"info"`: 정보 레벨 이상 출력 (기본값)
- `"warn"`: 경고 레벨 이상 출력
- `"error"`: 에러 레벨 이상 출력
- `"critical"`: 치명적 에러만 출력
- `"off"`: 모든 로그 비활성화

### 3. LIDAR 디버그 로그 제어
- `"LOG_ENABLE_LIDAR_DEBUG": "true"`: LIDAR 디버그 로그 활성화
- `"LOG_ENABLE_LIDAR_DEBUG": "false"`: LIDAR 디버그 로그 비활성화 (기본값)

## 사용 예시

### LIDAR 디버그 로그를 보고 싶을 때:
```json
{
    "logging": {
        "LOG_LEVEL": "debug",
        "LOG_ENABLE_LIDAR_DEBUG": "true"
    }
}
```

### 모든 로그를 끄고 싶을 때:
```json
{
    "logging": {
        "LOG_LEVEL": "off",
        "LOG_ENABLE_LIDAR_DEBUG": "false"
    }
}
```

### 에러와 경고만 보고 싶을 때:
```json
{
    "logging": {
        "LOG_LEVEL": "warn",
        "LOG_ENABLE_LIDAR_DEBUG": "false"
    }
}
```

## 코드에서의 사용
```cpp
// config에서 로그 레벨 가져오기
QString logLevel = config->get_log_level();
bool enableLidarDebug = config->get_log_enable_lidar_debug();

// LIDAR 디버그 로그 조건부 출력
if(config && config->get_log_enable_lidar_debug())
{
    spdlog::debug("[LIDAR] paired t=%.6f, idx: (%zu, %zu)...", ...);
}
```

## 주의사항
- config.json 파일을 수정한 후에는 프로그램을 재시작해야 설정이 적용됩니다.
- `LOG_ENABLE_LIDAR_DEBUG`는 `LOG_LEVEL`이 "debug" 이상일 때만 효과가 있습니다.

