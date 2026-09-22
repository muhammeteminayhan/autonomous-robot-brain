"""Shared pytest configuration for cross-package integration tests."""


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "integration: test requires a running ROS 2 graph; skipped otherwise",
    )
