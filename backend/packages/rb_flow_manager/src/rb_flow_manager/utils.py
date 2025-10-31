import ast
import inspect
import math
import operator
from typing import Any

ALLOWED_EVAL_OPS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.Pow: operator.pow,
    ast.Mod: operator.mod,
    ast.USub: operator.neg,
}

ALLOWED_EVAL_FUNCS = {
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
}

ALLOWED_MATH_NAMES = {
    "pi": math.pi,
    "e": math.e,
    "tau": getattr(math, "tau", 2 * math.pi),
    "inf": math.inf,
    "nan": math.nan,
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "sqrt": math.sqrt,
    "log": math.log,
    "log10": math.log10,
    "exp": math.exp,
}


def safe_eval_expr(expr: str, variables: dict[str, dict[str, Any]] | None = None) -> Any:
    """
    안전한 수식 평가:
    - 숫자/상수
    - 변수 참조
    - 사칙연산 / 나눗셈 / 거듭제곱
    - 제한된 함수 호출
    - math.접두사 접근 (math.sin, math.pi 등)
    """
    if variables is None:
        variables = {}

    def _eval(node: ast.AST) -> Any:
        if isinstance(node, ast.Constant):
            return node.value

        if isinstance(node, ast.Attribute):
            value = _eval(node.value)
            attr = node.attr

            # rb_var.local.*
            if value == "rb_var.local":
                if attr in variables.get("local", {}):
                    return variables["local"][attr]
                raise NameError(f"undefined rb_var.local.{attr}")

            # rb_var.global.*
            if value == "rb_var.global":
                if attr in variables.get("global", {}):
                    return variables["global"][attr]
                raise NameError(f"undefined rb_var.global.{attr}")

            # math.*
            if value is math:
                if attr in ALLOWED_MATH_NAMES:
                    return ALLOWED_MATH_NAMES[attr]
                raise ValueError(f"unknown math attribute: {attr}")

            raise ValueError("attribute access not allowed")

        if isinstance(node, ast.Name):
            # rb_var (접두사로만 사용)
            if node.id == "rb_var":
                return node.id

            # 허용된 함수
            if node.id in ALLOWED_EVAL_FUNCS:
                return ALLOWED_EVAL_FUNCS[node.id]

            # math 모듈
            if node.id == "math":
                return math

            # 허용된 상수
            if node.id in ALLOWED_MATH_NAMES:
                return ALLOWED_MATH_NAMES[node.id]

            # 그 외는 모두 문자열로 처리
            return node.id

        op_fn: Any = None

        if isinstance(node, ast.BinOp):
            op_fn = ALLOWED_EVAL_OPS.get(type(node.op), None)
            if op_fn is None:
                raise ValueError(f"unsupported operator: {type(node.op)}")
            return op_fn(_eval(node.left), _eval(node.right))

        if isinstance(node, ast.UnaryOp):
            op_fn = ALLOWED_EVAL_OPS.get(type(node.op), None)
            if op_fn is None:
                raise ValueError(f"unsupported unary operator: {type(node.op)}")
            return op_fn(_eval(node.operand))

        if isinstance(node, ast.Call):
            func_obj = _eval(node.func)

            if func_obj not in (*ALLOWED_EVAL_FUNCS.values(), *ALLOWED_MATH_NAMES.values()):
                raise ValueError("function not allowed")

            args = [_eval(a) for a in node.args]
            if node.keywords:
                raise ValueError("keyword arguments not allowed")

            return func_obj(*args)

        if isinstance(node, ast.List):
            return [_eval(e) for e in node.elts]

        raise ValueError(f"unsupported expression: {ast.dump(node)}")

    tree = ast.parse(expr, mode="eval")
    return _eval(tree.body)


def call_with_matching_args(func, **provided):
    """func의 시그니처를 보고 필요한 인자만 추려서 호출"""
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

    return func(**call_kwargs)
