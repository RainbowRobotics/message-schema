"""Rainbow Robotics SDK package.

이 패키지는 문서 빌드 중에 종속성을 방지하기 위해 lazy import를 사용하여 선택된 SDK 진입점을 제공합니다.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

__all__ = ["RBAmrSDK", "RBManipulateSDK", "RBBaseSDK"]

# 타입체커/IDE(자동완성)용: 여기서는 import 해도 런타임에 실행되지 않음
if TYPE_CHECKING:
    from .amr import RBAmrSDK
    from .base import RBBaseSDK
    from .manipulate import RBManipulateSDK


def __getattr__(name: str):
    if name == "RBManipulateSDK":
        from .manipulate import RBManipulateSDK  # noqa: PLC0415
        return RBManipulateSDK

    if name == "RBBaseSDK":
        from .base import RBBaseSDK  # noqa: PLC0415
        return RBBaseSDK

    if name == "RBAmrSDK":
        from .amr import RBAmrSDK  # noqa: PLC0415
        return RBAmrSDK

    raise AttributeError(name)


def __dir__() -> list[str]:
    # dir(rb_sdk) 결과가 __all__과 일치하게 (IDE/REPL 친화)
    return sorted(__all__)
