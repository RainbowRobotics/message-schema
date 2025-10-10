import json
from contextlib import AbstractContextManager, contextmanager, suppress
from dataclasses import dataclass
from importlib.resources import as_file, files
from pathlib import Path
from typing import Any

from fastapi import FastAPI


@dataclass
class _PersistEntry:
    cm: AbstractContextManager
    path: Path


def _ensure_state(app: FastAPI):
    if not hasattr(app.state, "_persist_entries"):
        app.state._persist_entries: dict[tuple[str, tuple[str, ...]], _PersistEntry] = {}
    return app.state._persist_entries


def open_resource_persist(app: FastAPI, *rel: str) -> Path:
    """
    패키지 리소스를 파일시스템 경로로 '영구화'하여 반환한다.
    - 최초 호출 시 as_file(...) 컨텍스트를 열고, 반환된 Path를 캐시한다.
    - 같은 (package, rel)로 재호출하면 기존 Path를 그대로 돌려준다.
    - 앱 종료 시 close_persist_resources(app)로 컨텍스트를 닫아준다.

    사용 예:
        p = open_resource_persist(app, "swagger-ui")   # 디렉터리
        app.mount("/swagger-ui", StaticFiles(directory=str(p)), name="swagger-ui")

        f = open_resource_persist(app, "data", "robot_models.json")  # 파일
        text = (f).read_text("utf-8")
    """
    entries = _ensure_state(app)
    key = ("rb_resources", tuple(rel))
    if key in entries:
        return entries[key].path

    res = files("rb_resources").joinpath(*rel)
    cm = as_file(res)
    path = cm.__enter__()  # 컨텍스트 오픈 (디렉터리/파일 모두 OK)

    entries[key] = _PersistEntry(cm=cm, path=path)
    return path


def close_persist_resources(app: FastAPI) -> None:
    """
    앱 종료 시 호출하여 open_resource_persist 로 연 컨텍스트를 모두 닫는다.
    """
    entries = getattr(app.state, "_persist_entries", None)
    if not entries:
        return
    for entry in entries.values():
        with suppress(Exception):
            entry.cm.__exit__(None, None, None)


@contextmanager
def open_resource(*rel: str):
    """
    with open_resource("schemas") as schemas_dir:
        app.mount("/schemas", StaticFiles(directory=str(schemas_dir)), name="schemas")

    with open_resource("config.json") as path:
        print("파일 경로:", path)
        print("파일 내용:", path.read_text(encoding="utf-8"))

    with open_resource("config.json5") as path:
        conf = Config.from_file(str(path))
        session = zenoh_open(conf)
    """
    res = files("rb_resources").joinpath(*rel)
    with as_file(res) as p:
        yield p


def read_text(*rel: str, encoding="utf-8") -> str:
    """
    텍스트/JSON 바로 로드할 때

    res = read_text("data", "robot_models.json")
    print(res, flush=True)
    """
    return files("rb_resources").joinpath(*rel).read_text(encoding=encoding)


def read_json_file(*rel: str, encoding="utf-8") -> Any:
    text = read_text(*rel, encoding=encoding)
    return json.loads(text)
