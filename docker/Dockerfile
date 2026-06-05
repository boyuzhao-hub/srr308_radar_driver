FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=humble
ARG DRIVER_REPO=https://github.com/boyuzhao-hub/srr308_radar_driver.git
ARG DRIVER_BRANCH=main

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/ros2_ws \
    ROS_DISTRO=${ROS_DISTRO}

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
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/%s/setup.bash\nsource %s/install/setup.bash\nexec "$@"\n' \
    "${ROS_DISTRO}" "${ROS_WS}" > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "radar_conti_ars408", "radar.launch.py"]