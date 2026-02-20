from __future__ import annotations

import os
import re
import sys
from pathlib import Path

PREFIX = os.environ.get("FLATBUF_PREFIX", "rb_flat_buffers")


def _collect_top_namespaces(root: Path) -> list[str]:
    # rb_flat_buffers/src/rb_flat_buffers 아래의 최상위 디렉토리들: program, flow_manager, IPC ...
    tops = []
    for p in root.iterdir():
        if p.is_dir() and p.name not in {"__pycache__", PREFIX}:
            tops.append(p.name)
    return sorted(tops)


def _prefix_import_lines(text: str, *, tops_alt: str) -> str:
    """
    import 문에서만:
      from program.X import ...  -> from rb_flat_buffers.program.X import ...
      import program.X          -> import rb_flat_buffers.program.X
    """
    lines = text.splitlines(keepends=True)
    out = []
    for ln in lines:
        ln = re.sub(rf"^(from)\s+({tops_alt})\.", rf"\1 {PREFIX}.\2.", ln)
        ln = re.sub(rf"^(import)\s+({tops_alt})\.", rf"\1 {PREFIX}.\2.", ln)
        out.append(ln)
    return "".join(out)


def _drop_missing_module_imports(root: Path, text: str) -> str:
    """
    from rb_flat_buffers.<ns>.<MOD> import ...
    에서 <ns>/<MOD>.py(.pyi)가 실제로 없으면 그 import 라인을 삭제한다.
    (flatc가 가끔 import는 만들고 파일은 안 만드는 케이스 대응)
    """

    patt = re.compile(
        rf"^from\s+{re.escape(PREFIX)}\.([A-Za-z0-9_]+)\.([A-Za-z0-9_]+)\s+import\s+[^\r\n]*\r?\n?",
        re.M,
    )

    def repl(m: re.Match[str]) -> str:
        ns = m.group(1)
        mod = m.group(2)
        ns_dir = root / ns
        if not ns_dir.is_dir():
            return ""  # 네임스페이스 폴더 자체가 없으면 제거
        if (ns_dir / f"{mod}.py").exists() or (ns_dir / f"{mod}.pyi").exists():
            return m.group(0)
        return ""  # 파일이 없으면 import 라인 제거

    return patt.sub(repl, text)


def _add_tclass_imports(root: Path, path: Path, text: str) -> str:
    """
    (옵션) SomethingT 타입을 쓰는데 import가 없으면 추가.
    네 기존 로직을 최대한 유지하되, namespace는 '# namespace: X'에서 추출.
    """
    mod_name = path.stem
    ns_match = re.search(r"^#\s*namespace:\s*([A-Za-z0-9_]+)\s*$", text, re.M)
    ns = ns_match.group(1) if ns_match else None
    if not ns:
        return text

    used_T = sorted(set(re.findall(r"\b([A-Z][A-Za-z0-9_]*?T)\b", text)))
    for tname in used_T:
        base = tname[:-1]
        if base == mod_name:
            continue

        # 해당 모듈 파일이 실제 존재하는 경우만 import 삽입
        if not ((root / ns / f"{base}.py").exists() or (root / ns / f"{base}.pyi").exists()):
            continue

        # 이미 base import 라인 있으면 T만 추가
        m = re.search(
            rf"^from\s+{re.escape(PREFIX)}\.{re.escape(ns)}\.{re.escape(base)}\s+import\s+([^\n]+)$",
            text,
            re.M,
        )
        if m:
            names = [x.strip() for x in m.group(1).split(",")]
            if tname not in names:
                names.append(tname)
                uniq = ", ".join(dict.fromkeys(names))
                text = re.sub(
                    rf"^from\s+{re.escape(PREFIX)}\.{re.escape(ns)}\.{re.escape(base)}\s+import\s+[^\n]+$",
                    f"from {PREFIX}.{ns}.{base} import {uniq}",
                    text,
                    count=1,
                    flags=re.M,
                )
            continue

        # base import가 없으면 상단에 삽입(대충 첫 import 블록 뒤)
        insert = f"from {PREFIX}.{ns}.{base} import {base}, {tname}\n"

        first_import = re.search(r"^(from|import)\s", text, re.M)
        if not first_import:
            text = insert + text
            continue

        # 첫 import 이후, 다음 빈 줄(없으면 첫 import 라인 끝)에 삽입
        start = first_import.start()
        blank = re.search(r"\n\s*\n", text[start:])
        if blank:
            hdr_end = start + blank.start() + 2
        else:
            hdr_end = text.find("\n", start) + 1

        text = text[:hdr_end] + insert + text[hdr_end:]

    return text


def _fix_pack_create_mismatch(text: str) -> str:
    """
    (옵션) Pack()이 return CreateX(...) 부르는데 Create 함수명이 다른 케이스를
    같은 파일의 첫 Create*로 맞춘다.
    """
    creates = re.findall(r"^def\s+(Create[A-Za-z0-9_]+)\s*\(", text, re.M)
    if not creates:
        return text
    preferred = creates[0]
    return re.sub(
        r"\breturn\s+(Create[A-Za-z0-9_]+)\s*\(",
        lambda m: f"return {preferred}(" if m.group(1) != preferred else m.group(0),
        text,
        flags=re.M,
    )


def _read_text_with_fallback(path: Path) -> tuple[str, str]:
    # flatc 산출물은 대부분 utf-8이지만, 윈도우 환경에서는 BOM/로컬 인코딩 케이스가 섞일 수 있다.
    for encoding in ("utf-8", "utf-8-sig", "cp949"):
        try:
            return path.read_text(encoding=encoding), encoding
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", b"", 0, 1, f"unable to decode: {path}")


def patch_generated_tree(root: Path) -> None:
    tops = _collect_top_namespaces(root)
    if not tops:
        return
    tops_alt = "|".join(re.escape(t) for t in tops)

    for path in root.rglob("*"):
        if path.suffix not in {".py", ".pyi"}:
            continue

        try:
            src, encoding = _read_text_with_fallback(path)
        except (OSError, UnicodeError) as exc:
            print(f"[patch_imports] skip unreadable file: {path} ({exc})", file=sys.stderr)
            continue

        # 1) import 라인에서만 프리픽스 부착
        new = _prefix_import_lines(src, tops_alt=tops_alt)

        # 2) 실제 파일 없는 모듈 import 제거 (SUB_TASK_INSER 같은 케이스 차단)
        new = _drop_missing_module_imports(root, new)

        # 3) (옵션) T-class import 보강
        new = _add_tclass_imports(root, path, new)

        # 4) (옵션) Pack/Create mismatch 교정
        new = _fix_pack_create_mismatch(new)

        if new != src:
            try:
                path.write_text(new, encoding=encoding, newline="")
            except OSError as exc:
                print(f"[patch_imports] skip unwritable file: {path} ({exc})", file=sys.stderr)


def main() -> None:
    if len(sys.argv) != 2:
        print("usage: patch_imports.py <FLAT_OUT_DIR>", file=sys.stderr)
        raise SystemExit(2)

    root = Path(sys.argv[1]).resolve()
    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        raise SystemExit(1)

    patch_generated_tree(root)


if __name__ == "__main__":
    main()
