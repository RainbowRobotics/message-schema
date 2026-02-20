#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import shutil
from dataclasses import dataclass
from pathlib import Path

SCALAR_MAP: dict[str, str] = {
    "bool": "bool",
    "byte": "int",
    "ubyte": "int",
    "short": "int",
    "ushort": "int",
    "int": "int",
    "uint": "int",
    "long": "int",
    "ulong": "int",
    "float": "float",
    "double": "float",
    "string": "str",
}


def pd_name(name: str) -> str:
    return name if name.endswith("PD") else f"{name}PD"


def to_camel(name: str) -> str:
    parts = name.split("_")
    if not parts:
        return name
    return parts[0] + "".join(p.capitalize() for p in parts[1:])


@dataclass
class FieldDef:
    name: str
    fbs_type: str
    default: str | None


@dataclass
class BlockDef:
    kind: str  # table | struct
    name: str
    fields: list[FieldDef]


@dataclass
class EnumDef:
    name: str
    members: list[tuple[str, str | None]]


@dataclass
class UnionDef:
    name: str
    members: list[str]


@dataclass
class SchemaUnit:
    source: Path
    namespace: str | None
    includes: list[str]
    blocks: list[BlockDef]
    enums: list[EnumDef]
    unions: list[UnionDef]


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//.*?$", "", text, flags=re.MULTILINE)
    return text


def parse_schema(path: Path) -> SchemaUnit:
    raw = strip_comments(path.read_text(encoding="utf-8"))

    namespace_match = re.search(r"\bnamespace\s+([A-Za-z0-9_.]+)\s*;", raw)
    namespace = namespace_match.group(1) if namespace_match else None

    includes = re.findall(r'\binclude\s+"([^"]+)"\s*;', raw)

    enums: list[EnumDef] = []
    for em in re.finditer(
        r"\benum\s+([A-Za-z_]\w*)\s*(?::\s*[A-Za-z_]\w*)?\s*\{(.*?)\}",
        raw,
        flags=re.DOTALL,
    ):
        name = em.group(1)
        body = em.group(2)
        members: list[tuple[str, str | None]] = []
        for item in body.split(","):
            piece = item.strip()
            if not piece:
                continue
            if "=" in piece:
                key, value = piece.split("=", 1)
                members.append((key.strip(), value.strip()))
            else:
                members.append((piece, None))
        enums.append(EnumDef(name=name, members=members))

    unions: list[UnionDef] = []
    for um in re.finditer(
        r"\bunion\s+([A-Za-z_]\w*)\s*\{(.*?)\}", raw, flags=re.DOTALL
    ):
        name = um.group(1)
        body = um.group(2)
        members = [x.strip() for x in body.split(",") if x.strip()]
        unions.append(UnionDef(name=name, members=members))

    blocks: list[BlockDef] = []
    for bm in re.finditer(
        r"\b(table|struct)\s+([A-Za-z_]\w*)\s*\{(.*?)\}", raw, flags=re.DOTALL
    ):
        kind = bm.group(1)
        name = bm.group(2)
        body = bm.group(3)
        fields: list[FieldDef] = []
        for line in body.split(";"):
            line = line.strip()
            if not line:
                continue
            line = re.sub(r"\([^)]*\)\s*$", "", line).strip()
            if ":" not in line:
                continue
            left, right = line.split(":", 1)
            field_name = left.strip()
            right = right.strip()
            default: str | None = None
            if "=" in right:
                ftype, dvalue = right.split("=", 1)
                right = ftype.strip()
                default = dvalue.strip()
            fields.append(FieldDef(name=field_name, fbs_type=right, default=default))
        blocks.append(BlockDef(kind=kind, name=name, fields=fields))

    return SchemaUnit(
        source=path,
        namespace=namespace,
        includes=includes,
        blocks=blocks,
        enums=enums,
        unions=unions,
    )


def snake_case(name: str) -> str:
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    s2 = re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1)
    return s2.replace("-", "_").lower()


def parse_vector_type(ftype: str) -> tuple[str, int | None] | None:
    m = re.fullmatch(r"\[\s*([A-Za-z0-9_.]+)\s*(?::\s*(\d+))?\s*\]", ftype)
    if not m:
        return None
    base = m.group(1)
    fixed = int(m.group(2)) if m.group(2) else None
    return base, fixed


def convert_type(fbs_type: str) -> tuple[str, str | None]:
    vec = parse_vector_type(fbs_type)
    if vec is not None:
        base, fixed_len = vec
        py_base = SCALAR_MAP.get(base, "dict")#pd_name(base.split(".")[-1]))
        if fixed_len is None:
            return f"list[{py_base}]", None
        return (
            f"list[{py_base}]",
            f"Field(default_factory=list, min_length={fixed_len}, max_length={fixed_len})",
        )

    py_scalar = SCALAR_MAP.get(fbs_type)
    if py_scalar:
        return py_scalar, None

    return "dict", None#pd_name(fbs_type.split(".")[-1]), None


def convert_default(py_type: str, raw_default: str | None) -> str:
    if raw_default is None:
        if py_type.startswith("list["):
            return "Field(default_factory=list)"
        if py_type in {"int", "float", "bool", "str"}:
            return "None"
        return "None"

    d = raw_default.strip()
    if d in {"true", "false"}:
        return "True" if d == "true" else "False"
    if re.fullmatch(r"-?\d+", d):
        return d
    if re.fullmatch(r"-?\d+\.\d*", d):
        return d
    if d.startswith('"') and d.endswith('"'):
        return d
    return "None"


def render_module(unit: SchemaUnit) -> str:
    lines: list[str] = []
    lines.append(
        "# AUTO-GENERATED by scripts/backend/fbs_to_pydantic.py. Derek said 'DO NOT EDIT. ㅡㅡ#'"
    )
    lines.append("from __future__ import annotations")
    lines.append("")
    lines.append("from enum import Enum")
    lines.append("from pydantic import BaseModel, ConfigDict, Field")
    lines.append("")

    include_modules = sorted(
        {snake_case(Path(x).stem) + "_models" for x in unit.includes}
    )
    if include_modules:
        for mod in include_modules:
            lines.append(f"from .{mod} import *  # noqa: F401,F403")
        lines.append("")

    if unit.namespace:
        lines.append(f'NAMESPACE = "{unit.namespace}"')
        lines.append("")

    for enum in unit.enums:
        enum_name = pd_name(enum.name)
        lines.append(f"class {enum_name}(Enum):")
        if not enum.members:
            lines.append("    pass")
        else:
            for key, value in enum.members:
                if value is None:
                    lines.append(f"    {key} = '{key}'")
                elif re.fullmatch(r"-?\d+", value):
                    lines.append(f"    {key} = {value}")
                else:
                    lines.append(f"    {key} = '{value}'")
        lines.append("")

    for uni in unit.unions:
        union_name = pd_name(uni.name)
        members = ", ".join(pd_name(m.split(".")[-1]) for m in uni.members)
        lines.append(f"# union {union_name}: {members}")
        lines.append(f"{union_name} = dict")
        lines.append("")

    for block in unit.blocks:
        block_name = pd_name(block.name)
        lines.append(f"class {block_name}(BaseModel):")
        lines.append("    model_config = ConfigDict(populate_by_name=True)")
        if not block.fields:
            lines.append("    pass")
            lines.append("")
            continue
        for field in block.fields:
            py_name = snake_case(field.name)
            camel_alias = to_camel(py_name)
            py_type, fixed_meta = convert_type(field.fbs_type)
            base_default = convert_default(py_type, field.default)
            field_args: list[str] = []
            if camel_alias != py_name:
                field_args.append(f'alias="{camel_alias}"')
            if field.name != py_name and field.name != camel_alias:
                field_args.append(f'serialization_alias="{field.name}"')

            if fixed_meta is not None:
                fixed_inner = fixed_meta[len("Field(") : -1]
                if fixed_inner:
                    field_args.append(fixed_inner)
                field_expr = (
                    f"Field({', '.join(field_args)})" if field_args else "Field()"
                )
                lines.append(f"    {py_name}: {py_type} = {field_expr}")
            elif base_default.startswith("Field(") and base_default.endswith(")"):
                default_inner = base_default[len("Field(") : -1]
                if default_inner:
                    field_args.append(default_inner)
                field_expr = (
                    f"Field({', '.join(field_args)})" if field_args else "Field()"
                )
                lines.append(f"    {py_name}: {py_type} = {field_expr}")
            elif base_default == "None":
                field_args_with_default = [*field_args, "default=None"]
                field_expr = f"Field({', '.join(field_args_with_default)})"
                lines.append(f"    {py_name}: {py_type} | None = {field_expr}")
            else:
                field_args_with_default = [*field_args, f"default={base_default}"]
                field_expr = f"Field({', '.join(field_args_with_default)})"
                lines.append(f"    {py_name}: {py_type} = {field_expr}")
        lines.append("")

    export_names = (
        [pd_name(e.name) for e in unit.enums]
        + [pd_name(u.name) for u in unit.unions]
        + [pd_name(b.name) for b in unit.blocks]
    )
    if export_names:
        lines.append("__all__ = [")
        for name in export_names:
            lines.append(f'    "{name}",')
        lines.append("]")
    else:
        lines.append("__all__ = []")

    lines.append("")
    return "\n".join(lines)


def update_init_file(output_dir: Path):
    module_files = sorted(p.stem for p in output_dir.glob("*_models.py"))
    subpackages = sorted(p.name for p in output_dir.iterdir() if p.is_dir())
    lines = [
        "# AUTO-GENERATED by scripts/fbs_to_pydantic.py. DO NOT EDIT.",
        "from __future__ import annotations",
        "",
    ]
    for pkg in subpackages:
        lines.append(f"from .{pkg} import *  # noqa: F401,F403")
    for stem in module_files:
        lines.append(f"from .{stem} import *  # noqa: F401,F403")
    lines.append("")
    lines.append("__all__ = []")
    lines.append("")
    (output_dir / "__init__.py").write_text("\n".join(lines), encoding="utf-8")


def update_init_files_recursively(output_root: Path):
    all_dirs = sorted(
        [p for p in output_root.rglob("*") if p.is_dir()],
        key=lambda p: len(p.parts),
        reverse=True,
    )
    all_dirs.append(output_root)
    # root should be processed last
    seen: set[Path] = set()
    for d in all_dirs:
        if d in seen:
            continue
        update_init_file(d)
        seen.add(d)


def generate(input_root: Path, output_root: Path):
    fbs_files = sorted(input_root.rglob("*.fbs"))
    if not fbs_files:
        raise RuntimeError(f"no .fbs files found in: {input_root}")

    if output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    for fbs in fbs_files:
        unit = parse_schema(fbs)
        rel_parent = fbs.relative_to(input_root).parent
        target_dir = output_root / rel_parent
        target_dir.mkdir(parents=True, exist_ok=True)
        module_name = snake_case(fbs.stem) + "_models.py"
        module_path = target_dir / module_name
        module_path.write_text(render_module(unit), encoding="utf-8")

    update_init_files_recursively(output_root)


def main():
    repo_root = Path(__file__).resolve().parents[2]

    parser = argparse.ArgumentParser(
        description="Generate pydantic models from FlatBuffers schemas"
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=repo_root / "schemas",
        help="directory containing .fbs files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repo_root
        / "backend"
        / "packages"
        / "rb_schemas"
        / "src"
        / "rb_schemas"
        / "fbs_models",
        help="output package directory for generated pydantic models",
    )
    args = parser.parse_args()

    generate(input_root=args.input.resolve(), output_root=args.output.resolve())
    print(f"[fbs_to_pydantic] generated from {args.input} -> {args.output}", flush=True)


if __name__ == "__main__":
    main()
