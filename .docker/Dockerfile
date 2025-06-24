ARG IMAGE_PREFIX=""
ARG ROS_DISTRO=noetic
ARG BASE_DOCKER_NAME=${ROS_DISTRO}
FROM ${IMAGE_PREFIX}ros:${BASE_DOCKER_NAME}-ros-base AS base

ENV TERM=xterm
ENV IN_DOCKER=1


###################################################################################
FROM base AS prepare
ARG MUJOCO_VERSIONS=2.3.6  # ARG must be declared in stage were it is used!

# Setup temporary ROS workspace
WORKDIR /root/ws_mujoco

# Copy MuJoCo ROS sources from docker context
COPY . src/mujoco_ros_pkgs

# Install packages required for prepare stage
RUN apt-get -q update && \
    apt-get -q -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get -q install --no-install-recommends -y curl git python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Install MuJoCo versions
RUN src/mujoco_ros_pkgs/.docker/mujoco_installer.sh

# Download upstream repo dependencies
RUN vcs import --shallow --input src/mujoco_ros_pkgs/.github/workflows/upstream.rosinstall src

# Create list of ros dependencies
RUN rosdep update $(test "${ROS_DISTRO}" = "noetic" && echo --rosdistro ${ROS_DISTRO})
RUN rosdep install -y --from-paths src --ignore-src --simulate > /tmp/deps.sh $(test "${ROS_DISTRO}" = "noetic" && echo --rosdistro ${ROS_DISTRO})
RUN chmod 755 /tmp/deps.sh


###################################################################################
FROM base AS final

COPY --from=prepare /root/mujoco /root/mujoco
COPY --from=prepare /tmp/deps.sh /tmp/deps.sh

RUN apt-get -q update && \
    apt-get -q -y dist-upgrade && \
    # Install common dependencies
    DEBIAN_FRONTEND=noninteractive apt-get -q install --no-install-recommends -y \
        libgles2-mesa-dev libosmesa6-dev libglfw3-dev \
        curl git sudo python3-vcstool \
        $(test "${ROS_DISTRO}" = "noetic" && echo "python3-catkin-tools" || echo "python3-colcon-common-extensions") \
        clang clang-format clang-tidy clang-tools \
        ccache && \
    # Install rosdep dependencies
    DEBIAN_FRONTEND=noninteractive /tmp/deps.sh && \
    # Cleanup apt cache
    rm -rf /var/lib/apt/lists/* && \
        # Globally disable git security
    git config --global --add safe.directory "*"
