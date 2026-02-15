FROM osrf/ros:jazzy-desktop

# 环境变量设置
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root/ws

# 1. 复制源码
COPY src/ src/

# 2. 安装依赖 (自动读取 package.xml)
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# 3. 强制赋予 Python 脚本可执行权限 (解决 WSL 权限丢失问题)
RUN chmod +x src/island_evo_core/src/*.py

# 4. 编译
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# 5. 配置自动 source (让你进容器就能直接跑)
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws/install/setup.bash" >> ~/.bashrc

# 6. 默认进入交互终端
CMD ["/bin/bash"]
