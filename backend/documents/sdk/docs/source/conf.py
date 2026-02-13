# Configuration file for the Sphinx documentation builder.

from __future__ import annotations

import sys
from pathlib import Path

STUBS = Path(__file__).resolve().parent / "_stubs"
sys.path.insert(0, str(STUBS))

project = "sdk-docs"
copyright = "2026, Derek Kim"
author = "Derek Kim"

# ---- path setup (핵심) ----
ROOT = Path(__file__).resolve().parents[4]  # backend/ 를 가리키도록
RB_SDK_SRC = ROOT / "packages" / "rb_sdk" / "src"
sys.path.insert(0, str(RB_SDK_SRC))

# ---- extensions ----
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.autosummary",
    "sphinx.ext.viewcode",
]


autosummary_generate = True
autodoc_typehints = "description"

autodoc_mock_imports = [
    "zenoh",
    "rb_sdk.amr_sdk.schema.amr_control_schema",
    "rb_sdk.amr_sdk.schema.amr_localization_schema",
    "rb_sdk.amr_sdk.schema.amr_map_schema",
    "rb_sdk.amr_sdk.schema.amr_move_schema",
    "rb_sdk.amr_sdk.schema.amr_setting_schema",
    "rb_sdk.amr_sdk.schema.amr_update_schema",
]

suppress_warnings = [
    "autodoc.mocked_object",
]

templates_path = ["_templates"]
exclude_patterns = []

language = "ko"

html_theme = "sphinx_rtd_theme"
_static_dir = Path(__file__).resolve().parent / "_static"
html_static_path = ["_static"] if _static_dir.exists() else []
