# backend/services/manipulate/freeze_entry.py
import os
import sys

from PyInstaller.__main__ import run as pyinstaller_run


def main():
    if len(sys.argv) < 3:
        print("usage: freeze_entry.py <entry.py> <outdir>", file=sys.stderr)
        sys.exit(2)

    entry = sys.argv[1]
    outdir = os.path.abspath(sys.argv[2])

    # 샌드박스 내부 쓰기 가능한 곳으로 모두 고정
    os.makedirs(outdir, exist_ok=True)
    workdir = os.path.join(outdir, ".work")
    cachedir = os.path.join(outdir, ".cache")
    os.makedirs(workdir, exist_ok=True)
    os.makedirs(cachedir, exist_ok=True)

    # macOS에서 홈 디렉터리 참조 막기
    os.environ["HOME"] = outdir
    # PyInstaller 캐시/설정 경로 환경변수 (6.x에서 인식)
    os.environ["PYINSTALLER_CONFIG_DIR"] = outdir
    os.environ["PYINSTALLER_CACHE_DIR"] = cachedir

    opts = [
        "--onefile",
        "--clean",
        f"--distpath={outdir}",
        f"--workpath={workdir}",
        f"--specpath={outdir}",
        "--name=run.bin",
        # 필요 시 hidden-import/collect-all 추가:
        # "--hidden-import", "pkg.mod",
        # "--collect-all", "zenoh",
        entry,
    ]
    print(f"[PyInstaller] {entry} -> {outdir}/run.bin")
    pyinstaller_run(opts)


if __name__ == "__main__":
    main()
