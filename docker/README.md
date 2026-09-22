# Docker environment

The native Ubuntu 24.04 install described in the top-level `README.md` is the
recommended way to develop this project. These files exist for CI and for
anyone who wants to reproduce the benchmark numbers on a different machine.

```bash
# Headless build + all unit tests (what CI runs)
UID=$(id -u) GID=$(id -g) docker compose -f docker/docker-compose.yml run --rm arb-ci

# Interactive shell with GUI (Gazebo / RViz) forwarded over X11
xhost +local:docker
UID=$(id -u) GID=$(id -g) docker compose -f docker/docker-compose.yml run --rm arb-gui
```

**Known limitation:** GPU-accelerated rendering inside the container is not
configured here. On a hybrid Intel/NVIDIA machine, either install the NVIDIA
Container Toolkit and add a `deploy.resources.devices` block, or run Gazebo
with `LIBGL_ALWAYS_SOFTWARE=1` and accept a low frame rate.
