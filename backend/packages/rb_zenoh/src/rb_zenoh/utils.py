# utils.py (수정)

import importlib
import inspect
import pkgutil


def recommend_cap(
    avg_bytes: float, mem_budget_mb: float, safety: float, min_cap: int = 10, max_cap: int = 200_000
) -> int:
    if avg_bytes <= 0:
        avg_bytes = 1.0
    bytes_budget = int(mem_budget_mb * 1024 * 1024 * safety)
    cap = max(min_cap, min(max_cap, bytes_budget // int(avg_bytes)))
    return int(cap)


async def invoke(cb, item):
    topic, mv, obj, attachment = item
    if inspect.iscoroutinefunction(cb):
        await cb(topic=topic, mv=mv, obj=obj, attachment=attachment)
    else:
        cb(topic=topic, mv=mv, obj=obj, attachment=attachment)


def resolve_object_api(FBClass) -> type | None:
    """
    flatc --python --gen-object-api 로 생성된 <Name>T 클래스를 찾는다.
    예: Person -> PersonT
    """
    mod = inspect.getmodule(FBClass)
    if not mod:
        return None
    return getattr(mod, FBClass.__name__ + "T", None)


def rough_size_of_fields(fields: dict) -> int:
    approx = 64
    for v in fields.values():
        if isinstance(v, str):
            approx += len(v) + 8
        elif isinstance(v, bytes | bytearray | memoryview):
            approx += len(v) + 16
        elif isinstance(v, int | float | bool):
            approx += 8
        elif isinstance(v, list | tuple):
            n = len(v)
            approx += 8 + n * 8
            if n and isinstance(v[0], str):
                approx += sum(len(s) for s in v if isinstance(s, str))
        elif isinstance(v, dict):
            approx += 64
        else:
            approx += 16
    return int(approx * 1.5)


def _to_camel(name: str) -> str:
    parts = []
    buf = ""
    for ch in name:
        if ch in ("_", "-"):
            if buf:
                parts.append(buf)
                buf = ""
        else:
            buf += ch
    if buf:
        parts.append(buf)
    return "".join(p[:1].upper() + p[1:] for p in parts)


def _find_T_class(root_module, camel_T: str, extra_roots: list[str] | None = None):
    """
    root_module: FBClass의 모듈 객체
    camel_T: 찾을 T 클래스명 (예: "PersonT")
    extra_roots: 추가 탐색할 패키지 루트 이름 리스트
    """
    seen = set()

    def walk_module(mod):
        if mod in seen:
            return None
        seen.add(mod)

        cls = getattr(mod, camel_T, None)
        if isinstance(cls, type):
            return cls

        for _, val in vars(mod).items():
            if inspect.ismodule(val) and hasattr(val, "__path__"):
                found = walk_module(val)
                if found:
                    return found

        if hasattr(mod, "__path__"):
            for _, name, _ in pkgutil.walk_packages(mod.__path__, mod.__name__ + "."):
                try:
                    m = importlib.import_module(name)
                except Exception:
                    continue
                found = walk_module(m)
                if found:
                    return found
        return None

    # 1차: 루트 모듈에서만 검색
    found_cls = walk_module(root_module)
    if found_cls:
        return found_cls

    # 2차: 추가 루트 모듈에서 검색
    for top in extra_roots or []:
        try:
            m = importlib.import_module(top)
        except ImportError:
            continue
        found_cls = walk_module(m)
        if found_cls:
            return found_cls

    return None


def set_attrs_loose(obj, fields: dict):

    root_module = importlib.import_module(obj.__class__.__module__)
    for k, v in fields.items():
        if isinstance(v, dict):
            camel_T = _to_camel(k) + "T"
            Tcls = _find_T_class(root_module, camel_T)
            if Tcls is None:
                raise TypeError(f"Field '{k}' expects {camel_T}, got dict. (자동 변환 실패)")
            child = Tcls()
            set_attrs_loose(child, v)
            setattr(obj, k, child)
        elif isinstance(v, list) and v and isinstance(v[0], dict):

            camel_T = _to_camel(k.rstrip("s")) + "T"
            Tcls = _find_T_class(root_module, camel_T)
            if Tcls is None:
                raise TypeError(f"Field '{k}' expects list[{camel_T}], got list[dict].")
            arr = []
            for d in v:
                ch = Tcls()
                set_attrs_loose(ch, d)
                arr.append(ch)
            setattr(obj, k, arr)
        else:
            setattr(obj, k, v)
