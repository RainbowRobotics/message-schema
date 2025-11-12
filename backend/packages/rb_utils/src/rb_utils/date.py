"""
[날짜 유틸리티]
"""
from datetime import UTC, date, datetime, timezone
from functools import lru_cache
from typing import Literal
from zoneinfo import ZoneInfo

_STRPTIME_FORMATS = (
    "%Y-%m-%dT%H:%M:%S%z",
    "%Y-%m-%d %H:%M:%S%z",
    "%Y-%m-%dT%H:%M:%S",
    "%Y-%m-%d %H:%M:%S",
    "%Y/%m/%d %H:%M:%S",
    "%Y-%m-%dT%H:%M",
    "%Y-%m-%d %H:%M",
    "%Y/%m/%d %H:%M",
    "%Y-%m-%d",
    "%Y/%m/%d",
    "%Y%m%d",
    "%Y%m%d%H%M%S",
    "%Y%m%d%H%M",
)


def is_valid_date(d: date | str | int):
    """
    - d: 날짜 문자열 또는 정수
    - 날짜 형식 검증
    """
    if isinstance(d, int):
        try:
            sec = timestamp_ms_to_seconds(int(d))
            if sec <= 0:
                return False

            datetime.fromtimestamp(sec, tz=UTC)
            return True
        except (ValueError, OverflowError, OSError):
            return False

    if isinstance(date, str):
        s = date.strip()
        try:
            s2 = s[:-1] + "+00:00" if s.endswith("Z") else s
            datetime.fromisoformat(s2)
            return True
        except (ValueError, OverflowError, OSError):
            pass

        for fmt in _STRPTIME_FORMATS:
            try:
                datetime.strptime(s, fmt)
                return True
            except (ValueError, OverflowError, OSError):
                continue
        return False

    return False

def timestamp_ms_to_seconds(timestamp: int, *, tz: timezone = UTC):
    """
    - timestamp: 밀리초 단위 타임스탬프
    - tz: 타임존 (기본: UTC)
    - 밀리초 단위 타임스탬프를 초 단위 타임스탬프로 변환
    """
    if timestamp > 10**12:
        return datetime.fromtimestamp(timestamp / 1000.0, tz=tz).timestamp()
    return datetime.fromtimestamp(timestamp, tz=tz).timestamp()

def is_has_time(s: str) -> bool:
    """
    - s: 날짜 문자열
    - 날짜 문자열에 시간이 포함되어 있는지 확인
    """
    s = s.strip()
    if ":" in s or "T" in s:
        return True

    digits = "".join(ch for ch in s if ch.isdigit())
    return len(digits) > 8

def parse_date_with_time(
    mode: Literal["start", "end"], d: str | int | datetime, *, tz: timezone = UTC
):
    """
    - mode: 날짜 타입 (기본: "start")
    - d: 날짜 문자열 또는 정수 또는 datetime
    - tz: 타임존 (기본: UTC)
    - 날짜 문자열 또는 정수 또는 datetime를 datetime 객체로 변환
    """
    if not is_valid_date(d):
        raise ValueError("Invalid date")

    if isinstance(d, datetime):
        dt = d

        dt = dt.replace(tzinfo=tz) if dt.tzinfo is None else dt.astimezone(tz)

        has_time = not (dt.hour == 0 and dt.minute == 0 and dt.second == 0 and dt.microsecond == 0)
    elif isinstance(d, int):
        sec = timestamp_ms_to_seconds(int(d))
        dt = datetime.fromtimestamp(sec, tz=tz)
        has_time = True
    else:
        s = d.strip()
        parsed = None

        try:
            s2 = s[:-1] + "+00:00" if s.endswith("Z") else s
            parsed = datetime.fromisoformat(s2)
        except (ValueError, OverflowError, OSError):
            parsed = None

        if parsed is None:
            for fmt in _STRPTIME_FORMATS:
                try:
                    parsed = datetime.strptime(s, fmt)
                    break
                except (ValueError, OverflowError, OSError):
                    parsed = None

        if parsed is None:
            raise ValueError("Unparseable date string")

        parsed = parsed.replace(tzinfo=tz) if parsed.tzinfo is None else parsed.astimezone(tz)
        dt = parsed

        if is_has_time(s):
            has_time = True
        else:
            has_time = not (
                dt.hour == 0 and dt.minute == 0 and dt.second == 0 and dt.microsecond == 0
            )

    if not has_time:
        if mode == "start":
            dt = datetime(dt.year, dt.month, dt.day, 0, 0, 0, 0, tzinfo=tz)
        else:
            dt = datetime(dt.year, dt.month, dt.day, 23, 59, 59, 999999, tzinfo=tz)

    return dt

# === Yujin Added ===


# DateTime으로 변환 (날짜만 들어오면 그날 00:00:00 timezone 시간으로 간주)
def ensure_datetime(dt: date | datetime, tz:str) -> datetime:
    """
    - dt: 날짜 또는 datetime
    - tz: 타임존
    - 날짜 또는 datetime를 datetime 객체로 변환
    """
    if not tz or not isinstance(tz, str):
        raise TypeError("timezone must be provided")
    tz = ZoneInfo(tz)

    if isinstance(dt, datetime):
        return dt.replace(tzinfo=tz) if dt.tzinfo is None else dt.astimezone(tz)

    if isinstance(dt, date):
        return dt.combine(dt, datetime.min.time()).replace(tzinfo=tz)

    raise TypeError("dt must be date or datetime")

@lru_cache(maxsize=128)
def _tz(tz_name: str) -> ZoneInfo:
    if not tz_name or not isinstance(tz_name, str):
        raise TypeError("timezone must be a non-empty string (e.g., 'Asia/Seoul').")
    try:
        return ZoneInfo(tz_name)
    except Exception as e:
        raise ValueError(f"Unknown timezone: {tz_name}") from e


# 날짜를 timezone 변환하여 datetime으로 반환
def convert_dt(
    local: date | datetime,
    in_tz: str,
    out_tz: str,
) -> datetime:
    """
    - local: 날짜 또는 datetime
    - in_tz: 입력 타임존
    - out_tz: 출력 타임존
    - 날짜 또는 datetime를 타임존 변환
    """
    tz_in = _tz(in_tz)
    tz_out = _tz(out_tz)
    print(f"[convert_dt] tz_in: {tz_in}, tz_out: {tz_out}")

    # datetime 들어온 경우
    if isinstance(local, datetime):
        dt_in = local.replace(tzinfo=tz_in) if local.tzinfo is None else local.astimezone(tz_in)
        dt_out = dt_in.astimezone(tz_out)
        return dt_out

    # date 들어온 경우 (날짜만 있을 때)
    if isinstance(local, date):
        # 그 날짜의 00:00:00 in_tz 로 만든 후 out_tz 로 변환
        dt_in = datetime.combine(local, datetime.min.time()).replace(tzinfo=tz_in)
        dt_out = dt_in.astimezone(tz_out)
        return dt_out
    raise TypeError("local must be date or datetime")


def get_current_dt_yyyymmddhhmmss(tz: str) -> str:
    """
    - tz: 타임존
    - 현재 날짜+시간을 YYYYMMDDHHMMSS로 변환하여 반환
    """
    return datetime.now(ZoneInfo(tz)).strftime("%Y%m%d%H%M%s")




