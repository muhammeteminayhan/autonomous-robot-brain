"""Static well-formedness checks for every XML asset in the repository.

These run in milliseconds and without ROS, and they catch a family of mistakes
that otherwise surface as an opaque parser error deep inside a launch file.

The double-hyphen rule is the one worth calling out: XML forbids ``--`` inside
a comment, because that sequence is what terminates the comment token. Prose
written with a double hyphen as an em dash therefore produces

    XML parsing error: not well-formed (invalid token): line 33, column 67

which names a line whose content looks perfectly innocent. This has bitten this
project twice, so it is now a test.
"""

from __future__ import annotations

import re
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SEARCH_ROOTS = (REPO_ROOT / "ros2_ws" / "src",)
XML_SUFFIXES = (".xacro", ".urdf", ".sdf", ".xml")

#: RViz configs use a .rviz extension but are YAML documents, not XML.
YAML_SUFFIXES = (".yaml", ".yml", ".rviz")
SKIP_DIRS = {"build", "install", "log", "__pycache__", ".git"}

#: ``<!--`` ... ``-->``; DOTALL so multi-line comments are captured whole.
_COMMENT_RE = re.compile(rb"<!--(.*?)-->", re.DOTALL)


def _files_with(suffixes: tuple[str, ...], roots=SEARCH_ROOTS) -> list[Path]:
    found: list[Path] = []
    for root in roots:
        for path in root.rglob("*"):
            if not path.is_file() or path.suffix not in suffixes:
                continue
            if SKIP_DIRS & set(path.relative_to(REPO_ROOT).parts):
                continue
            found.append(path)
    return sorted(found)


def _xml_files() -> list[Path]:
    return _files_with(XML_SUFFIXES)


def _yaml_files() -> list[Path]:
    return _files_with(YAML_SUFFIXES, roots=SEARCH_ROOTS + (REPO_ROOT / "configs",))


def test_xml_assets_are_discovered():
    """A silent empty file list would make every other check vacuous."""
    files = _xml_files()
    assert len(files) >= 4, f"expected several XML assets, found {len(files)}: {files}"


@pytest.mark.parametrize("path", _xml_files(), ids=lambda p: p.name)
def test_xml_is_well_formed(path: Path):
    """Every XML asset must parse."""
    try:
        ET.parse(path)
    except ET.ParseError as exc:  # pragma: no cover - the message is the point
        pytest.fail(f"{path.relative_to(REPO_ROOT)}: {exc}")


@pytest.mark.parametrize("path", _xml_files(), ids=lambda p: p.name)
def test_no_double_hyphen_inside_comments(path: Path):
    """``--`` inside an XML comment is illegal; use an em dash in prose."""
    data = path.read_bytes()
    offenders: list[str] = []
    for match in _COMMENT_RE.finditer(data):
        body = match.group(1)
        if b"--" in body:
            line = data[: match.start()].count(b"\n") + 1
            snippet = next(
                (ln.decode("utf-8", "replace").strip()
                 for ln in body.splitlines() if b"--" in ln),
                "",
            )
            offenders.append(f"comment starting at line {line}: {snippet!r}")
    assert not offenders, (
        f"{path.relative_to(REPO_ROOT)} has '--' inside an XML comment "
        f"(illegal; use an em dash):\n  " + "\n  ".join(offenders)
    )


@pytest.mark.parametrize("path", _yaml_files(), ids=lambda p: p.name)
def test_yaml_is_parseable(path: Path):
    """Every YAML asset must parse, including .rviz files (which are YAML)."""
    import yaml

    try:
        yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:  # pragma: no cover - the message is the point
        pytest.fail(f"{path.relative_to(REPO_ROOT)}: {exc}")


def test_yaml_assets_are_discovered():
    files = _yaml_files()
    assert len(files) >= 8, f"expected several YAML assets, found {len(files)}"
