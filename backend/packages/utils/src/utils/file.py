import re
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
