# Copyright (c) 2026 Emin Ayhan. MIT License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check that every Python docstring satisfies PEP 257.

    D213 is added to the ignore list because ament's two linters contradict
    each other on it, and something has to choose.

    D212 ("summary on the first line") and D213 ("summary on the second line")
    are mutually exclusive: no docstring can satisfy both. ament_flake8 runs
    flake8-docstrings, whose default convention selects D212 and leaves D213
    off; ament_pep257 runs pydocstyle with an explicit --ignore list, which
    leaves every unlisted check ON, D213 included. A multi-line docstring
    therefore fails one linter or the other whatever it does, and this package
    only avoided the collision while it had no multi-line docstrings at all.

    D212 wins here because it is what every other Python file in this
    repository already does, and what the C++ doc comments do.
    """
    rc = main(argv=['.', 'test', '--add-ignore', 'D213'])
    assert rc == 0, 'Found code style errors / warnings'
