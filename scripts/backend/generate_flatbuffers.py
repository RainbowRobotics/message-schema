#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


def clean_output_dir(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for child in out_dir.iterdir():
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def generate_flatbuffers(flatc: str, schemas_root: Path, out_dir: Path) -> None:
    schema_files = sorted(schemas_root.rglob("*.fbs"))
    if not schema_files:
        raise SystemExit(f"no .fbs files found under: {schemas_root}")

    base_cmd = [
        flatc,
        "--python",
        "--gen-object-api",
        "--gen-all",
        "--python-typing",
        "--python-gen-numpy",
        "-o",
        str(out_dir),
    ]
    for schema in schema_files:
        subprocess.run([*base_cmd, str(schema)], check=True)


def write_init_files(out_dir: Path) -> None:
    for directory in sorted([out_dir, *[p for p in out_dir.rglob("*") if p.is_dir()]]):
        (directory / "__init__.py").touch()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate Python FlatBuffers files.")
    parser.add_argument("--flatc", default="flatc", help="flatc executable path")
    parser.add_argument("--schemas-root", type=Path, required=True, help="schemas root path")
    parser.add_argument("--out-dir", type=Path, required=True, help="output package path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    schemas_root = args.schemas_root.resolve()
    out_dir = args.out_dir.resolve()

    clean_output_dir(out_dir)
    generate_flatbuffers(args.flatc, schemas_root, out_dir)
    write_init_files(out_dir)


if __name__ == "__main__":
    main()
