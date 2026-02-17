FROM osrf/ros:jazzy-desktop

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    DEBIAN_FRONTEND=noninteractive

WORKDIR /root/ws

# 1) Copy source
COPY src/ src/

# 2) Install dependencies
RUN apt-get update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/*

# 3) Make python scripts executable (do not fail if none exist)
RUN chmod +x src/island_evo_core/scripts/*.py 2>/dev/null || true \
 && chmod +x src/island_evo_core/src/*.py 2>/dev/null || true

# 4) Build workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --symlink-install

# 5) Auto-source on shell entry
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "source /root/ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
