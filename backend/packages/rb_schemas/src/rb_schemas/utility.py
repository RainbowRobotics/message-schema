from typing import Any, get_type_hints

from pydantic import BaseModel, Field, create_model
from pydantic.fields import FieldInfo
from pydantic_core import PydanticUndefined


def Omit(base: type[BaseModel], *omit: str) -> type[BaseModel]:
    omit_set = set(omit)
    annotations = get_type_hints(base, include_extras=True)
    fields: dict[str, tuple[Any, Any]] = {}

    for name, ann in annotations.items():
        if name in omit_set:
            continue
        fi: FieldInfo = base.model_fields[name]

        if (
            fi.default is not PydanticUndefined or fi.default_factory is not None
        ):  # 기본값이 있으면(심지어 None이어도) 그대로
            default = fi
        elif fi.default_factory is not None:
            default = Field(default_factory=fi.default_factory)
        else:
            default = ...  # type: ignore

        fields[name] = (ann, default)

    return create_model(  # type: ignore
        f"{base.__name__}Omit_{'_'.join(sorted(omit_set))}",
        __module__=base.__module__,
        **fields,
    )


def Pick(base: type[BaseModel], *include: str) -> type[BaseModel]:
    include_set = set(include)
    annotations = get_type_hints(base, include_extras=True)
    fields: dict[str, tuple[Any, Any]] = {}

    for name, ann in annotations.items():
        if name not in include_set:
            continue
        fi: FieldInfo = base.model_fields[name]

        if fi.default is not PydanticUndefined:
            default = fi.default
        elif fi.default_factory is not None:
            default = Field(default_factory=fi.default_factory)
        else:
            default = ...

        fields[name] = (ann, default)

    return create_model(  # type: ignore
        f"{base.__name__}Pick_{'_'.join(sorted(include_set))}",
        __module__=base.__module__,
        **fields,
    )
