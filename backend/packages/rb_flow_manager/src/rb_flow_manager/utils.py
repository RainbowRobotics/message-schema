from typing import Any

from .context import ExecutionContext


def _resolve_arg_scope_value(v: Any, ctx: ExecutionContext) -> Any:
    """
    - dict  : value들을 재귀 처리
    - list  : 요소들을 재귀 처리
    - "$parent.xxx": ctx.lookup("xxx")로 한 번만 부모에서 값 찾아서 치환
    기타     : 그대로 반환
    """

    # dict이면 내부까지 재귀
    if isinstance(v, dict):
        return {k: _resolve_arg_scope_value(val, ctx) for k, val in v.items()}

    # list면 요소들 재귀
    if isinstance(v, list):
        return [_resolve_arg_scope_value(item, ctx) for item in v]

    # "$parent.xxx" 처리
    if isinstance(v, str) and v.startswith("$parent."):
        attr = v[len("$parent.") :]

        # 🔴 여기서 부모 스코프에서 attr을 찾는다
        try:
            parent_val = ctx.lookup(attr)
        except KeyError as e:
            # 여기서 명확히 터뜨리도록 해두면, 어디서 안 잡히는지 바로 알 수 있다.
            raise RuntimeError(f"failed to resolve parent pointer: {v}") from e

        # parent_val 안에도 dict/list/$parent.가 있을 수 있으니 한 번 더 재귀
        return _resolve_arg_scope_value(parent_val, ctx)

    # 나머지는 그대로
    return v
