import json
from contextlib import contextmanager
from importlib.resources import as_file, files
from typing import Any


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
    res = files("rb_resources").joinpath("data", *rel)
    with as_file(res) as p:
        yield p


# 텍스트/JSON 바로 로드할 때
def read_text(*rel: str, encoding="utf-8") -> str:
    return files("rb_resources").joinpath("data", *rel).read_text(encoding=encoding)


def read_json_file(*rel: str, encoding="utf-8") -> Any:
    text = read_text(*rel, encoding=encoding)
    return json.loads(text)
