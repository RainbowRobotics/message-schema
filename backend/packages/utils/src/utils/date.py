from datetime import UTC, datetime, timezone
from typing import Literal

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


def is_valid_date(date: str | int):
    if isinstance(date, int):
        try:
            sec = timestamp_ms_to_seconds(int(date))
            if sec <= 0:
                return False

            datetime.fromtimestamp(sec, tz=UTC)
            return True
        except Exception:
            return False

    if isinstance(date, str):
        s = date.strip()
        try:
            s2 = s[:-1] + "+00:00" if s.endswith("Z") else s
            datetime.fromisoformat(s2)
            return True
        except Exception:
            pass

        for fmt in _STRPTIME_FORMATS:
            try:
                datetime.strptime(s, fmt)
                return True
            except Exception:
                continue
        return False

    return False


def timestamp_ms_to_seconds(timestamp: int, *, timezone: timezone = UTC):
    if timestamp > 10**12:
        return datetime.fromtimestamp(timestamp / 1000.0, tz=timezone).timestamp()
    return datetime.fromtimestamp(timestamp, tz=timezone).timestamp()


def is_has_time(s: str) -> bool:
    s = s.strip()
    if ":" in s or "T" in s:
        return True

    digits = "".join(ch for ch in s if ch.isdigit())
    return len(digits) > 8


def parse_date_with_time(
    type: Literal["start", "end"], date: str | int | datetime, *, timezone: timezone = UTC
):
    if not is_valid_date(date):
        raise ValueError("Invalid date")

    if isinstance(date, datetime):
        dt = date

        dt = dt.replace(tzinfo=timezone) if dt.tzinfo is None else dt.astimezone(timezone)

        has_time = not (dt.hour == 0 and dt.minute == 0 and dt.second == 0 and dt.microsecond == 0)
    elif isinstance(date, int):
        sec = timestamp_ms_to_seconds(int(date))
        dt = datetime.fromtimestamp(sec, tz=timezone)
        has_time = True
    else:
        s = date.strip()
        parsed = None

        try:
            s2 = s[:-1] + "+00:00" if s.endswith("Z") else s
            parsed = datetime.fromisoformat(s2)
        except Exception:
            parsed = None

        if parsed is None:
            for fmt in _STRPTIME_FORMATS:
                try:
                    parsed = datetime.strptime(s, fmt)
                    break
                except Exception:
                    parsed = None

        if parsed is None:
            raise ValueError("Unparseable date string")

        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone)
        else:
            parsed = parsed.astimezone(timezone)

        dt = parsed

        if is_has_time(s):
            has_time = True
        else:
            has_time = not (
                dt.hour == 0 and dt.minute == 0 and dt.second == 0 and dt.microsecond == 0
            )

    if not has_time:
        if type == "start":
            dt = datetime(dt.year, dt.month, dt.day, 0, 0, 0, 0, tzinfo=timezone)
        else:
            dt = datetime(dt.year, dt.month, dt.day, 23, 59, 59, 999999, tzinfo=timezone)

    return dt
