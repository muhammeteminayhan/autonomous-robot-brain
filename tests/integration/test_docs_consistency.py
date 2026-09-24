"""Keep the documentation's internal links and indices honest.

A broken link in a README is the kind of defect that survives indefinitely,
because nothing executes it. These checks run in milliseconds and without ROS,
and they exist because a real one survived for six phases: the learning-notes
index listed ``phase-05-degerlendirme.md``, that file was never written, and
nobody following the link would have found out why.

Three properties:

* every relative link in a Markdown file points at something that exists
* the learning-notes index and the learning-notes directory agree, in BOTH
  directions -- a note that exists but is unlisted is as invisible as one that
  is listed but absent
* every phase that has produced code has a note
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
NOTES_DIR = REPO_ROOT / "docs" / "learning-notes"
NOTES_INDEX = NOTES_DIR / "README.md"

#: Inline Markdown links: [text](target). Reference-style links and bare URLs
#: are deliberately not matched -- the first are not used here and the second
#: cannot be checked without network access.
LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")

#: Matched against a link target to decide whether it is ours to verify.
EXTERNAL = re.compile(r"^(https?:|mailto:|#)")


def markdown_files() -> list[Path]:
    files = [REPO_ROOT / "README.md"]
    files += sorted((REPO_ROOT / "docs").rglob("*.md"))
    files += sorted((REPO_ROOT / "tests").rglob("*.md"))
    files += sorted((REPO_ROOT / "docker").rglob("*.md"))
    return [f for f in files if f.is_file()]


@pytest.mark.parametrize("path", markdown_files(),
                         ids=lambda p: str(p.relative_to(REPO_ROOT)))
def test_every_relative_link_resolves(path: Path):
    """No Markdown file may link to a path that does not exist."""
    broken: list[str] = []
    for target in LINK.findall(path.read_text(encoding="utf-8")):
        if EXTERNAL.match(target.strip()):
            continue
        # Strip any fragment; the file's existence is what is checked here.
        clean = target.split("#", 1)[0].strip()
        if not clean:
            continue
        if not (path.parent / clean).resolve().exists():
            broken.append(target)
    assert not broken, (
        f"{path.relative_to(REPO_ROOT)} links to missing paths:\n  "
        + "\n  ".join(broken))


def _indexed_notes() -> set[str]:
    return set(re.findall(r"phase-\d+-[a-z0-9-]+\.md", NOTES_INDEX.read_text()))


def _present_notes() -> set[str]:
    return {p.name for p in NOTES_DIR.glob("phase-*.md")}


def test_the_notes_index_lists_nothing_that_is_missing():
    """The failure this whole file exists for.

    The index listed a Phase 5 note for six phases while the file did not
    exist. Nothing pointed it out, because an index is prose until something
    checks it.
    """
    missing = _indexed_notes() - _present_notes()
    assert not missing, f"indexed but absent from {NOTES_DIR}: {sorted(missing)}"


def test_the_notes_index_lists_everything_that_is_present():
    """And the other direction: an unlisted note is an invisible note."""
    unlisted = _present_notes() - _indexed_notes()
    assert not unlisted, f"present but not in the index: {sorted(unlisted)}"


def test_every_phase_with_a_package_or_script_has_a_note():
    """A phase that produced code must have produced an explanation of it.

    The phase numbers come from the notes themselves rather than from a list
    kept here, so adding a phase does not mean editing this test -- it means
    the new phase's note has to exist before the gap closes.
    """
    numbers = sorted(int(re.match(r"phase-(\d+)", n).group(1))
                     for n in _present_notes())
    assert numbers, "no learning notes found at all"
    # Contiguous from zero: a hole means a phase was documented on either side
    # of one that was not.
    expected = list(range(numbers[0], numbers[-1] + 1))
    holes = sorted(set(expected) - set(numbers))
    assert not holes, f"no learning note for phase(s) {holes}"
