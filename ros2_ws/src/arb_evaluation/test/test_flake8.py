# Copyright (c) 2026 Emin Ayhan. MIT License.

import os

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check that every Python file satisfies the flake8 profile.

    The profile is this package's .flake8, which is ament's with the
    single-quote preference (Q000) removed and nothing else relaxed. See the
    comment in that file for why.
    """
    config = os.path.join(os.path.dirname(__file__), os.pardir, '.flake8')
    rc, errors = main_with_errors(argv=['--config', config])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
