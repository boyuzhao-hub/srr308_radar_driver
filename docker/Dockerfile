ARG L4T_VERSION=l4t-r36.3.0
FROM dustynv/ros:humble-ros-base-${L4T_VERSION}

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=humble
ARG DRIVER_REPO=https://github.com/boyuzhao-hub/srr308_radar_driver.git
ARG DRIVER_BRANCH=main

ARG CACHE_BUST=1

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/ros2_ws \
    ROS_DISTRO=${ROS_DISTRO} \
    # Use CycloneDDS as the default RMW implementation
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

RUN apt-get update 2>/dev/null || true && \
    apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    nano \
    python3-colcon-common-extensions \
    python3-rosdep \
    iproute2 \
    can-utils \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}

RUN mkdir -p src && \
    git clone --depth 1 --branch ${DRIVER_BRANCH} ${DRIVER_REPO} src/srr308_radar_driver


RUN apt-get update && \
    rosdep init 2>/dev/null || true && \
    rosdep update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build/ log/


RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/%s/setup.bash\nsource %s/install/setup.bash\nexec "$@"\n' \
    "${ROS_DISTRO}" "${ROS_WS}" > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]