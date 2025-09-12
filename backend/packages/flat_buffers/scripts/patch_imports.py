from __future__ import annotations
import io, os, re, sys, tokenize
from bisect import bisect_right
from pathlib import Path

# flat_buffer / flat_buffers 등 선택 가능
PREFIX = os.environ.get("FLATBUF_PREFIX", "flat_buffers")


def protect_ranges(src: str):
    b = io.BytesIO(src.encode("utf-8"))
    protected = []
    for tok in tokenize.tokenize(b.readline):
        if tok.type in (tokenize.STRING, tokenize.COMMENT):
            s = _offset_of(src, tok.start)
            e = _offset_of(src, tok.end)
            protected.append((s, e))
    protected.sort()
    merged = []
    for s, e in protected:
        if not merged or merged[-1][1] < s:
            merged.append([s, e])
        else:
            merged[-1][1] = max(merged[-1][1], e)
    return [(s, e) for s, e in merged]


def _offset_of(src: str, lc):
    line, col = lc
    off = 0
    for i, ln in enumerate(src.splitlines(keepends=True), start=1):
        if i == line:
            return off + col
        off += len(ln)
    return off


def overlaps(protected, s, e):
    i = bisect_right(protected, (s, float("inf"))) - 1
    if i >= 0 and protected[i][1] > s:
        return True
    return bool(i + 1 < len(protected) and protected[i + 1][0] < e)


def add_prefix_and_fix(root: Path):
    tops = sorted(
        p.name for p in root.iterdir() if p.is_dir() and p.name not in {"__pycache__", PREFIX}
    )
    if not tops:
        return
    tops_alt = "|".join(re.escape(t) for t in tops)
    patt_from = re.compile(rf"(?<!{re.escape(PREFIX)}\.)\b({tops_alt})\.", re.M)

    for path in root.rglob("*"):
        if path.suffix not in {".py", ".pyi"}:
            continue
        src = path.read_text(encoding="utf-8")
        prot = protect_ranges(src)

        # A) 프리픽스 치환
        out, last = [], 0
        for m in patt_from.finditer(src):
            s, e = m.span()
            if overlaps(prot, s, e):
                continue
            out.append(src[last:s])
            out.append(f"{PREFIX}.{m.group(1)}.")
            last = e
        out.append(src[last:])
        new = "".join(out)

        # B) T-class import 보강 (자기 자신 모듈 제외)
        #   모듈 이름/네임스페이스 추정
        mod_name = path.stem  # N_DIN_u 등
        ns_match = re.search(r"^#\s*namespace:\s*([A-Za-z0-9_]+)\s*$", new, re.M)
        ns = ns_match.group(1) if ns_match else "IPC"

        used_T = sorted(set(re.findall(r"\b([A-Z][A-Za-z0-9_]*?T)\b", new)))
        for tname in used_T:
            base = tname[:-1]
            # 자기 자신이면 건너뜀 (self import 방지)
            if base == mod_name:
                continue
            # 이미 base import 라인 있으면 T 추가
            m = re.search(
                rf"^from\s+{re.escape(PREFIX)}\.{ns}\.{re.escape(base)}\s+import\s+([^\n]+)$",
                new,
                re.M,
            )
            if m:
                names = [x.strip() for x in m.group(1).split(",")]
                if tname not in names:
                    names.append(tname)
                    uniq = ", ".join(dict.fromkeys(names))
                    new = re.sub(
                        rf"^from\s+{re.escape(PREFIX)}\.{ns}\.{re.escape(base)}\s+import\s+[^\n]+$",
                        f"from {PREFIX}.{ns}.{base} import {uniq}",
                        new,
                        count=1,
                        flags=re.M,
                    )
                continue
            # base import가 없으면 상단 import 블록 뒤에 삽입
            insert = f"from {PREFIX}.{ns}.{base} import {base}, {tname}\n"
            hdr_end = 0
            m2 = re.search(r"(?:^|\n)(?:import\s|from\s)", new)
            if m2:
                m3 = re.search(r"\n\s*\n", new[m2.start() :])
                if m3:
                    hdr_end = m2.start() + m3.start() + 2
            new = new[:hdr_end] + insert + new[hdr_end:]

        # C) Pack() → Create* 함수명 불일치 교정
        #    파일에 정의된 Create 함수명을 수집하고, Pack이 부르는 이름이
        #    그중 하나와 다르면 가장 유력한(동일 파일의 첫 Create) 이름으로 교체
        creates = re.findall(r"^def\s+(Create[A-Za-z0-9_]+)\s*\(", new, re.M)
        if creates:
            preferred = creates[0]  # 보통 struct 모듈은 하나뿐

            def repl_call(m):
                name = m.group(1)
                return m.group(0).replace(name, preferred) if name != preferred else m.group(0)

            new = re.sub(
                r"\breturn\s+(Create[A-Za-z0-9_]+)\s*\(",
                lambda m: f"return {preferred}(" if m.group(1) != preferred else m.group(0),
                new,
            )

        if new != src:
            path.write_text(new, encoding="utf-8")


def main():
    if len(sys.argv) != 2:
        print("usage: patch_imports.py <FLAT_OUT_DIR>", file=sys.stderr)
        sys.exit(2)
    root = Path(sys.argv[1]).resolve()
    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        sys.exit(1)
    add_prefix_and_fix(root)


if __name__ == "__main__":
    main()
