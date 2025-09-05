#!/usr/bin/env python3
import glob
import os
import sys

# Python 3.11+: tomllib 표준. 3.10 이하면 tomli 설치 후 import tomli as tomllib
try:
    import tomllib
except Exception:  # pragma: no cover
    import tomli as tomllib  # type: ignore

ROOT = os.path.dirname(os.path.dirname(__file__))
SEARCH_DIRS = [
    os.path.join(ROOT, "backend", "packages"),
    os.path.join(ROOT, "backend", "services"),
]


def read_deps(pyproject_path: str) -> list[str]:
    with open(pyproject_path, "rb") as f:
        data = tomllib.load(f)
    proj = data.get("project") or {}
    deps = proj.get("dependencies") or []
    # 문자열만 취급, 환경마커/extra는 그대로 둔다
    out = []
    for d in deps:
        if not isinstance(d, str):
            continue
        d = d.strip()
        if not d or d.startswith("#"):
            continue
        out.append(d)
    return out


def main():
    all_deps: set[str] = set()
    for base in SEARCH_DIRS:
        for py in glob.glob(os.path.join(base, "**", "pyproject.toml"), recursive=True):
            all_deps.update(read_deps(py))

    # 같은 배포명이 여러 버전으로 표기되면 가장 구체적인 것 우선(==, ~= 등) — 간단 정책
    # 충돌 정교 처리 원하면 여기에 우선순위 규칙 더해도 됨.
    # 우선은 그대로 쓰고, uv가 해석/해결.

    out_in = os.path.join(ROOT, "backend", "requirements.in")
    with open(out_in, "w", encoding="utf-8") as f:
        for d in sorted(all_deps, key=str.lower):
            f.write(d + "\n")

    print(f"Wrote {out_in}. Next:")
    print("  uv pip compile backend/requirements.in -o backend/requirements.txt")


if __name__ == "__main__":
    sys.exit(main())
