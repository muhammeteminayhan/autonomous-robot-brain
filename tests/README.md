# Tests

| Location | Kind | Runner |
|---|---|---|
| `ros2_ws/src/<pkg>/test/` | Unit tests (gtest for C++, pytest for Python) | `colcon test` |
| `tests/integration/` | System tests that span packages | `pytest tests/integration` |

Unit tests live inside their package so `colcon test` discovers them and so a
package stays self-contained. Integration tests live here because they inspect
the *whole* graph and belong to no single package.

```bash
./scripts/run_tests.sh                 # all unit tests
pytest tests/integration -v            # static architecture checks
pytest tests/integration -v -m integration   # requires a running stack
```
