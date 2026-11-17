# =============================================================================
# Reproducible build/test environment for Autonomous Robot Brain.
#
# The primary development path for this project is a NATIVE install on
# Ubuntu 24.04 (see README "Installation"), because Gazebo Harmonic and RViz2
# both want direct GPU access and X11, which is simpler outside a container.
#
# This image exists for the two jobs containers are genuinely better at:
#   1. CI -- build + unit tests on a clean machine, no GUI needed.
#   2. Reproducibility -- pinning the exact toolchain a reviewer needs to
#      reproduce the benchmark numbers in results/.
# =============================================================================
FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Toolchain and library dependencies -------------------------------------
# The list lives in docker/apt-packages.txt so that this image and the GitHub
# Actions workflow cannot drift apart. Copied in before the install so the
# layer cache invalidates exactly when the dependency set changes.
COPY docker/apt-packages.txt /tmp/apt-packages.txt
RUN apt-get update \
    && sed -e 's/#.*//' -e '/^[[:space:]]*$/d' /tmp/apt-packages.txt \
       | xargs apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages.txt

# --- Non-root user so files written into the mounted workspace stay editable -
ARG USERNAME=arb
ARG UID=1000
ARG GID=1000
RUN if getent passwd ${UID} >/dev/null; then userdel -r "$(getent passwd ${UID} | cut -d: -f1)" || true; fi && \
    groupadd -f -g ${GID} ${USERNAME} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} && \
    echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash" \
      >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspace

COPY --chown=${UID}:${GID} docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
