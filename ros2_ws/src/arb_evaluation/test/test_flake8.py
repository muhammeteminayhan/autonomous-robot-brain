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

    SCOPE IS THE WORKING DIRECTORY. ament_flake8 lints `.` and nothing tells it
    which package it belongs to, so this must be run with the cwd set to the
    package -- which is what `colcon test` does. Invoked as
    `pytest ros2_ws/src/arb_evaluation/test` from the repo root it lints the
    WHOLE REPO, including scripts/ and tests/ which are not ROS packages and do
    not follow this profile, and then fails on files it has no business
    checking. Run it through colcon.
    """
    config = os.path.join(os.path.dirname(__file__), os.pardir, '.flake8')
    rc, errors = main_with_errors(argv=['--config', config])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
