ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base

ARG MUJOCO_VERSIONS=[2.3.6]

ENV TERM xterm
ENV MUJOCO_DIR=/root/mujoco/2.3.6

# Setup temporary ROS workspace
WORKDIR /root/ws_mujoco

# Copy MuJoCo ROS sources from docker context
COPY . src/mujoco_ros_pkgs

# Combined commands for less stages
RUN \
    apt-get -q update && \
    apt-get -q -y dist-upgrade && \
    #
    # Base dependencies
    apt-get -q install --no-install-recommends -y \
        wget curl git sudo python3-wstool \
        # Build tools
        python3-catkin-tools \
        clang clang-format clang-tidy clang-tools \
        ccache && \
    #
    # Install MuJoCo Versions
    MUJOCO_VERSIONS=${MUJOCO_VERSIONS} ./src/mujoco_ros_pkgs/.docker/mujoco_installer.sh && \
    #
    # Add paths to generic mujoco installs
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$MUJOCO_DIR/lib:/usr/lib/nvidia\n \
    export CPATH=\$CPATH:\$MUJOCO_DIR/include\n \
    export LIBRARY_PATH=\$LIBRARY_PATH:\$MUJOCO_DIR/lib" >> ${HOME}/.bashrc && \
    #
    # Download upstream repo dependencies
    wstool init --shallow src ./src/mujoco_ros_pkgs/.github/workflows/upstream.rosinstall && \
    #
    # Download all dependencies
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Remove sources from the container
    rm -rf src && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/* && \
    #
    # Globally disable git security
    git config --global --add safe.directory "*"

ENV IN_DOCKER 1
