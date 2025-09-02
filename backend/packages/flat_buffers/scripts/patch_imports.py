from __future__ import annotations
import re
import sys
from pathlib import Path
import tokenize
import io
from bisect import bisect_right

def protect_ranges(src: str):
    b = io.BytesIO(src.encode("utf-8"))
    protected = []
    for tok in tokenize.tokenize(b.readline):
        if tok.type in (tokenize.STRING, tokenize.COMMENT):
            start_off = _offset_of(src, tok.start)
            end_off   = _offset_of(src, tok.end)
            protected.append((start_off, end_off))

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
    offs = 0

    for i, ln in enumerate(src.splitlines(keepends=True), start=1):
        if i == line:
            return offs + col
        offs += len(ln)
    return offs

def overlaps(protected, s, e):
    i = bisect_right(protected, (s, float("inf"))) - 1

    if i >= 0 and protected[i][1] > s:
        return True
    if i + 1 < len(protected) and protected[i + 1][0] < e:
        return True
    return False

def main():
    if len(sys.argv) != 2:
        print("usage: patch_imports.py <FLAT_OUT_DIR>", file=sys.stderr)
        sys.exit(2)

    root = Path(sys.argv[1]).resolve()

    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        sys.exit(1)

    tops = sorted([
        p.name for p in root.iterdir()
        if p.is_dir() and p.name not in {"__pycache__", "flat_buffers"}
    ])

    if not tops:
        return


    tops_alt = "|".join(re.escape(t) for t in tops)
    pattern = re.compile(rf"(?<!flat_buffers\.)\b({tops_alt})\.", flags=re.M)

    for py in root.rglob("*.py"):
        src = py.read_text(encoding="utf-8")

        prot = protect_ranges(src)
        out = []
        last = 0

        for m in pattern.finditer(src):
            s, e = m.span()
            if overlaps(prot, s, e):
                continue

            out.append(src[last:s])
            top = m.group(1)
            out.append(f"flat_buffers.{top}.")
            last = e
        out.append(src[last:])

        new = "".join(out)
        if new != src:
            py.write_text(new, encoding="utf-8")

if __name__ == "__main__":
    main()
