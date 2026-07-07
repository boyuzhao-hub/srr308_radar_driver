#!/bin/bash
set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_WS=${ROS_WS:-/root/ros2_ws}
RADAR_SOURCE_DIR=${RADAR_SOURCE_DIR:-${ROS_WS}/src/srr308_radar_driver}
RADAR_COLCON_BASE_PATHS=${RADAR_COLCON_BASE_PATHS:-${RADAR_SOURCE_DIR}/src}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

source_ros_setup() {
  if [ -f "/opt/ros/${ROS_DISTRO}/install/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/install/setup.bash"
  fi

  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  fi
}

source_ros_setup

if [ "${ROSDEP_ON_ENTRYPOINT:-0}" = "1" ]; then
  rosdep update
  rosdep install -y -r -q \
    --from-paths "${RADAR_COLCON_BASE_PATHS}" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}"
fi

if [ "${AUTO_BUILD:-1}" = "1" ]; then
  if [ ! -d "${RADAR_COLCON_BASE_PATHS}" ]; then
    echo "Radar package path not found: ${RADAR_COLCON_BASE_PATHS}"
    exit 1
  fi

  echo "Building SRR308 radar workspace from ${RADAR_COLCON_BASE_PATHS}"
  cd "${ROS_WS}"
  colcon build \
    --symlink-install \
    --base-paths "${RADAR_COLCON_BASE_PATHS}" \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
fi

if [ -f "${ROS_WS}/install/local_setup.bash" ]; then
  source "${ROS_WS}/install/local_setup.bash"
fi

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

echo "SRR308 Radar ROS2 Docker Image"
echo "------------------------------"
echo "ROS distro: ${ROS_DISTRO}"
echo "DDS middleware: ${RMW_IMPLEMENTATION}"
echo "ROS 2 workspace: ${ROS_WS}"
echo "Radar source: ${RADAR_SOURCE_DIR}"
echo "ROS 2 Domain ID: ${ROS_DOMAIN_ID}"
echo "Local IPs: $(hostname -I)"
echo "---"
echo "Available radar packages:"
ros2 pkg list | grep -E "^(radar_conti|socketcan|nav2_dynamic|kf_hungarian)" || true
echo "------------------------------"
echo "To start the radar node:"
echo "  ros2 launch radar_conti_srr308 radar.launch.py"
echo "To start the SocketCAN bridge:"
echo "  ros2 launch socketcan_adapter_ros socketcan_bridge_launch.py can_interface:=can0"
echo "------------------------------"

cd "${ROS_WS}"
exec "$@"
