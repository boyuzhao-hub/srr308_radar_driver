# srr308_radar_driver
This is the ROS2 based radar driver for Continental SRR308, which is based on Continental ARS408 driver from FH Aachen University of Applied Sciences[Lecture: Perception_Radar](https://gitlab.com/ApexAI/autowareclass2020/-/blob/bc1206171c96907977b0f5a8f4e3bc039ce61ae6/lectures/09_Perception_Radar/Radar-Hands-On.md) and Polymathrobotics.

## Docker development image

Build the Ubuntu 22.04 ROS 2 Humble image:

```shell
docker build \
  -t srr308-radar-driver:desktop-humble \
  -f docker/Dockerfile.desktop-humble \
  .
```

Run it with the host checkout mounted into the container workspace:

```shell
docker run -it \
  --name srr308_radar_driver \
  --privileged \
  --network=host \
  --ipc=host \
  --pid=host \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AUTO_BUILD=1 \
  -v /mnt/nvme/Projects/radar_ros2_ws/src/srr308_radar_driver:/root/ros2_ws/src/srr308_radar_driver:rw \
  -v srr308_radar_build:/root/ros2_ws/build \
  -v srr308_radar_install:/root/ros2_ws/install \
  -v srr308_radar_log:/root/ros2_ws/log \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  srr308-radar-driver:desktop-humble
```

Restart the stopped container:

```shell
docker start -ai srr308_radar_driver
```

Open another shell in a running container:

```shell
docker exec -it srr308_radar_driver bash
```

Bring up SocketCAN before launching the radar driver:

```shell
./scripts/setup_can.sh can0 can1
```

The default bitrate is `500000` with `restart-ms=100`. Override them when
needed:

```shell
CAN_BITRATE=500000 CAN_RESTART_MS=100 ./scripts/setup_can.sh can0 can1
```

Check CAN state:

```shell
ip -details -statistics link show can0
ip -details -statistics link show can1
```

The radar launch defaults to no automatic CAN writes with `enable_can_tx: false`.
This keeps the driver from sending startup filter/configuration frames. Manual
configuration services are controlled separately by `enable_manual_can_tx`.
Keep `enable_can_tx: false` while debugging CAN bus-off issues, and call the
configuration service one radar at a time.

Manual configuration services are node-local. Use `radar_can0` for sensor IDs
`0` and `1`, and `radar_can1` for sensor ID `2`:

```shell
ros2 service call /radar_can0/set_radar_configuration radar_conti_ars408_msgs/srv/TriggerSetCfg "{sensor_id: 0}"
ros2 service call /radar_can0/set_radar_configuration radar_conti_ars408_msgs/srv/TriggerSetCfg "{sensor_id: 1}"
ros2 service call /radar_can1/set_radar_configuration radar_conti_ars408_msgs/srv/TriggerSetCfg "{sensor_id: 2}"
```

Use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` in every ROS 2
container that must exchange topics.
