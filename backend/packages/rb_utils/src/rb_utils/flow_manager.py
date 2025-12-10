import ast
import asyncio
import inspect
import math
import operator
import re
from collections.abc import Callable, Coroutine
from concurrent.futures import Future
from typing import Any

# --- 간단 테이블 (함수/상수/연산자) -----------------------------------------
BIN_OPS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.FloorDiv: operator.floordiv,
    ast.Pow: operator.pow,
    ast.Mod: operator.mod,
}
UNARY_OPS: dict[type[ast.unaryop], Callable] = {ast.USub: operator.neg, ast.UAdd: operator.pos}
CMP_OPS: dict[type[ast.cmpop], Callable] = {
    ast.Eq: operator.eq,
    ast.NotEq: operator.ne,
    ast.Lt: operator.lt,
    ast.LtE: operator.le,
    ast.Gt: operator.gt,
    ast.GtE: operator.ge,
}
ALLOWED_FUNCS = {
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "sqrt": math.sqrt,
    "log": math.log,
    "log10": math.log10,
    "exp": math.exp,
    "abs": abs,
    "max": max,
    "min": min,
    "pow": pow,
    "sum": sum,
    "round": round,
    "int": int,
    "float": float,
    "str": str,
    "len": len,
    "range": range,
}
ALLOWED_CALLABLES = set(ALLOWED_FUNCS.values())
ALLOWED_CONSTS = {
    "pi": math.pi,
    "e": math.e,
    "tau": getattr(math, "tau", 2 * math.pi),
    "inf": math.inf,
    "nan": math.nan,
}


def safe_eval_expr(
    expr: Any,
    variables: dict[str, dict[str, Any]] | None = None,
    get_global_variable: Callable[[str], Any] | None = None,
) -> Any:
    if not isinstance(expr, str):
        return expr

    vars_ = variables or {"local": {}}
    s = expr.strip()
    if not s:
        return ""

    # --- 증감(식 전체일 때만 허용) ---------------------------------------------
    inc_post = re.match(r"^([A-Za-z_]\w*)\s*\+\+$", s)  # a++
    dec_post = re.match(r"^([A-Za-z_]\w*)\s*--$", s)  # a--
    inc_pre = re.match(r"^\+\+\s*([A-Za-z_]\w*)$", s)  # ++a
    dec_pre = re.match(r"^--\s*([A-Za-z_]\w*)$", s)  # --a
    match = inc_post or dec_post or inc_pre or dec_pre
    if match:
        name = match.group(1)
        exists = name in vars_.get("local", {}) or name in vars_.get("global", {})
        cur: int | None = (
            vars_.get("local", {}).get(name)
            if name in vars_.get("local", {})
            else vars_.get("global", {}).get(name, 0)
        )

        if cur is None:
            raise ValueError(f"variable {name} not found")

        if inc_post:
            vars_.setdefault("local", {})[name] = cur + 1 if exists else 1
            return cur
        if dec_post:
            vars_.setdefault("local", {})[name] = cur - 1 if exists else -1
            return cur
        if inc_pre:
            v = cur + 1 if exists else 1
            vars_.setdefault("local", {})[name] = v
            return v
        if dec_pre:
            v = cur - 1 if exists else -1
            vars_.setdefault("local", {})[name] = v
            return v

    # --- 내부 평가기 ------------------------------------------------------------
    def _eval(node: ast.AST) -> Any:
        if isinstance(node, ast.Constant):
            return node.value

        if isinstance(node, ast.Name):
            name = node.id
            if name == "true":
                return True
            if name == "false":
                return False
            if name == "null":
                return None
            if name in ALLOWED_FUNCS:  # 함수 이름
                return ALLOWED_FUNCS[name]
            if name in ALLOWED_CONSTS:  # 수학 상수
                return ALLOWED_CONSTS[name]
            if name == "math":  # math.* 전용
                return "math"
            if name in vars_.get("local", {}):
                return vars_["local"].get(name)

            if name.startswith("RB_"):
                global_val = get_global_variable(name) if get_global_variable is not None else None
                if global_val is not None:
                    return global_val
                return name

            return name  # 모르는 식별자는 문자열로 취급(원하면 여기서 NameError)

        if isinstance(node, ast.Attribute):
            base = _eval(node.value)
            attr = node.attr
            if isinstance(base, dict) and attr in base:
                return base[attr]
            if base == "math":
                if attr in ALLOWED_CONSTS:
                    return ALLOWED_CONSTS[attr]
                if attr in ALLOWED_FUNCS and hasattr(math, attr):
                    return getattr(math, attr)
                raise ValueError(f"unknown math attribute: {attr}")
            raise ValueError(f"attribute access not allowed: {type(base).__name__}.{attr}")

        if isinstance(node, ast.Subscript):
            seq = _eval(node.value)
            sl = node.slice
            if isinstance(sl, ast.Slice):
                lo = _eval(sl.lower) if sl.lower else None
                up = _eval(sl.upper) if sl.upper else None
                st = _eval(sl.step) if sl.step else None
                return seq[slice(lo, up, st)]
            return seq[_eval(sl)]

        if isinstance(node, ast.BinOp):
            try:
                bin_fn = BIN_OPS[type(node.op)]  # get() 쓰지 말고 [] 사용 → Optional 제거
            except KeyError:
                raise ValueError(f"unsupported operator: {type(node.op)}") from None
            return bin_fn(_eval(node.left), _eval(node.right))

        if isinstance(node, ast.UnaryOp):
            try:
                un_fn = UNARY_OPS[type(node.op)]  # 변수명 분리(= 타입 분리)
            except KeyError:
                raise ValueError(f"unsupported unary operator: {type(node.op)}") from None
            return un_fn(_eval(node.operand))

        if isinstance(node, ast.Compare):
            left = _eval(node.left)
            for op, comp in zip(node.ops, node.comparators, strict=False):  # strict 인자 빼
                try:
                    cmp_fn = CMP_OPS[type(op)]  # Optional 제거
                except KeyError:
                    raise ValueError(f"unsupported comparison: {type(op)}") from None
                right = _eval(comp)
                if not cmp_fn(left, right):
                    return False
                left = right
            return True

        if isinstance(node, ast.BoolOp):
            if isinstance(node.op, ast.And):
                return all(_eval(v) for v in node.values)
            if isinstance(node.op, ast.Or):
                return any(_eval(v) for v in node.values)

        if isinstance(node, ast.Call):
            fn = _eval(node.func)
            if fn not in ALLOWED_CALLABLES:
                raise ValueError(f"function not allowed: {fn}")
            if node.keywords:
                raise ValueError("keyword arguments not allowed")
            args = [_eval(a) for a in node.args]
            return fn(*args)

        if isinstance(node, ast.List):
            return [_eval(e) for e in node.elts]
        if isinstance(node, ast.Tuple):
            return tuple(_eval(e) for e in node.elts)
        if isinstance(node, ast.Dict):
            return {
                (_eval(k) if k is not None else None): _eval(v)
                for k, v in zip(node.keys, node.values, strict=False)
            }

        if isinstance(node, ast.IfExp):
            return _eval(node.body) if _eval(node.test) else _eval(node.orelse)

        raise ValueError(f"unsupported expression: {ast.dump(node)}")

    # --- 단순 대입: a = <expr> (한 문장만 허용) ---------------------------------
    try:
        mod = ast.parse(s, mode="exec")
        if len(mod.body) == 1 and isinstance(mod.body[0], ast.Assign):
            tgt = mod.body[0].targets[0]
            if not isinstance(tgt, ast.Name):
                raise ValueError("only simple assignment like `a = <expr>` is allowed")
            val = _eval(mod.body[0].value)
            vars_.setdefault("local", {})[tgt.id] = val
            return val
    except SyntaxError:
        pass  # 표현식일 수 있으니 아래로 진행

    # --- 일반 표현식 ------------------------------------------------------------
    try:
        tree = ast.parse(s, mode="eval")
        return _eval(tree.body)
    except Exception:
        raise ValueError(f"failed to evaluate expression: {expr}") from None


def _pick_target_loop(func, provided: dict) -> asyncio.AbstractEventLoop | None:
    bound_self = getattr(func, "__self__", None)
    if bound_self is not None:
        loop = getattr(bound_self, "loop", None)
        if isinstance(loop, asyncio.AbstractEventLoop):
            return loop

    loop = provided.get("loop")
    if isinstance(loop, asyncio.AbstractEventLoop):
        return loop

    return None


def call_with_matching_args(func, **provided):
    """func의 시그니처를 보고 필요한 인자만 추려서 호출 (동기/비동기 모두 지원, 루프 안전)"""
    sig = inspect.signature(func)
    call_kwargs = {}

    for name, param in sig.parameters.items():
        if name in provided:
            call_kwargs[name] = provided[name]
        elif param.default is inspect.Parameter.empty and param.kind not in (
            inspect.Parameter.VAR_POSITIONAL,
            inspect.Parameter.VAR_KEYWORD,
        ):
            raise TypeError(f"Missing required parameter: {name}")

    result: Any = func(**call_kwargs)

    async def _await_awaitable(a):
        return await a

    if inspect.isawaitable(result):
        target_loop = _pick_target_loop(func, provided)

        if target_loop is not None:
            coro: Coroutine[Any, Any, Any] = result if inspect.iscoroutine(result) else _await_awaitable(result)
            future: Future[Any] = asyncio.run_coroutine_threadsafe(coro, target_loop)
            return future.result()

        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.run(_await_awaitable(result))

        task: asyncio.Task = running.create_task(_await_awaitable(result))
        return task

    # 3) 동기 함수면 결과 그대로 반환
    return result


def eval_value(
    value: Any,
    variables: dict[str, dict[str, Any]] | None = None,
    get_global_variable: Callable[[str], Any] | None = None,
) -> Any:
    # 1) 문자열이면 → 기존 safe_eval_expr 호출
    if isinstance(value, str):
        return safe_eval_expr(
            value,
            variables=variables,
            get_global_variable=get_global_variable,
        )

    # 2) 리스트면 → 각 요소를 재귀적으로 평가
    if isinstance(value, list):
        return [
            eval_value(v, variables=variables, get_global_variable=get_global_variable)
            for v in value
        ]

    # 3) 튜플도 동일
    if isinstance(value, tuple):
        return tuple(
            eval_value(v, variables=variables, get_global_variable=get_global_variable)
            for v in value
        )

    # 4) 딕셔너리면 → value만 재귀적으로 평가 (key는 그대로)
    if isinstance(value, dict):
        return {
            k: eval_value(v, variables=variables, get_global_variable=get_global_variable)
            for k, v in value.items()
        }

    # 5) 나머지(int/float/bool/None 등)는 그대로
    return value
