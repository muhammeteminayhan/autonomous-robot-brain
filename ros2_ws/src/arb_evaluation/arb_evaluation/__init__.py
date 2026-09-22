"""Offline analysis toolkit for the Autonomous Robot Brain stack.

This package is deliberately *not* part of the runtime control loop. It reads
recorded data (rosbags / CSV) after the fact and produces the metrics, plots
and benchmark tables that appear in ``results/``. Keeping evaluation out of the
live pipeline is what lets it use ground truth without ever contaminating the
estimator -- see docs/architecture.md, "Ground truth isolation".
"""

__version__ = "0.1.0"
