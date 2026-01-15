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
]

templates_path = ["_templates"]
exclude_patterns = []

language = "ko"

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
