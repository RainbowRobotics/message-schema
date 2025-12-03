import re
import sys
from pathlib import Path
from urllib.parse import quote


def content_disposition_header(filename: str) -> str:
    safe_fname = filename.replace('"', "'")
    quoted = quote(filename, safe="")
    return f"attachment; filename=\"{safe_fname}\"; filename*=UTF-8''{quoted}"


def sanitize_filename(
    name: str,
    default: str = "export.csv",
    max_name_len: int = 100,
) -> str:
    if not name or not isinstance(name, str):
        return default

    name = name.replace("\x00", "").strip()
    name = re.sub(r'[<>:"/\\|?*\x00-\x1F]', "_", name)

    if len(name) > max_name_len:
        name = name[:max_name_len]

    return name


def get_env_path() -> Path | None:
    """현재 파일 기준으로 위로 올라가면서 .env 를 찾는다."""
    # PyInstaller로 빌드된 경우: _MEIPASS 기준
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        base = Path(meipass)
        candidate = base / ".env"
        if candidate.exists():
            return candidate

    # 개발 환경: __file__ 기준으로 위로 타고 올라가며 .env 검색
    cur = Path(__file__).resolve()

    for p in [cur, *cur.parents]:
        env_path = p / ".env"
        print("env_path >>>", env_path, p, env_path.is_file(), env_path.exists())
        if env_path.is_file():
            return env_path

    # 못 찾으면 None
    return None
