# srr308_radar_driver
This is the ROS2 based radar driver for Continental SRR308, adapted from the Continental radar driver from FH Aachen University of Applied Sciences[Lecture: Perception_Radar](https://gitlab.com/ApexAI/autowareclass2020/-/blob/bc1206171c96907977b0f5a8f4e3bc039ce61ae6/lectures/09_Perception_Radar/Radar-Hands-On.md) and Polymathrobotics.

## Docker development image

Build the Ubuntu 22.04 ROS 2 Humble image:

```shell
docker build \
  -t srr308-radar:humble-ubuntu22.04 \
  -f docker/Dockerfile.desktop-humble \
  .
```

Run it with the host checkout mounted into the container workspace:

```shell
 docker run -it \
  --name srr308_radar_container \
  --privileged \
  --network=host \
  --ipc=host \
  --pid=host \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AUTO_BUILD=1 \
  -v /mnt/nvme/Projects/radar_ros2_ws/src/srr308_radar_driver:/root/ros2_ws/src/srr308_radar_driver:rw \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  srr308-radar:humble-ubuntu22.04
```

Restart the stopped container:

```shell
docker start -ai srr308_radar_container
```

Open another shell in a running container:

```shell
docker exec -it srr308_radar_container bash
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

The launch file disables lifecycle bond monitoring (`bond_timeout: 0.0`) and
still uses lifecycle services for configure/activate transitions. This keeps
the basic driver free of bond timers and avoids a shutdown race with the
SocketCAN receive thread.

Cluster visualization uses one `SPHERE_LIST` marker per radar. The following
ROS parameters control visibility without changing the radar hardware
configuration:

- `cluster_marker_lifetime_sec` (built-in default `2.0`, current YAML `0.5`):
  how long the last cluster marker remains visible.
- `cluster_marker_scale` (built-in default `0.8`, current YAML `0.6`): marker
  diameter in metres.
- `cluster_scan_timeout_ms` (default `200`): publish a pending scan if no next
  Status frame arrives.
- `max_cluster_frames_per_scan` (default `512`): memory-safety limit for a
  malformed or status-less continuous stream. Reaching it publishes the raw
  frames collected so far as a partial chunk.

The static `<radar_link>/fov` MarkerArray is independent of the radar filter
configuration. It is published immediately on activation, every three seconds
for late volatile subscribers, and with reliable/transient-local QoS. The
`<radar_link>/fov_filter` marker is different: it represents confirmed active
distance and azimuth filters, and is deleted when those filters are inactive.
In RViz, select `MarkerArray` for `/<radar_link>/marker_array` and
`/<radar_link>/fov`, but select `Marker` for `/<radar_link>/fov_filter`.

Every received Cluster General frame is retained in CAN receive order, including
repeated Cluster IDs. A scan is published when the next Status arrives, on
timeout, or while deactivating the driver. `complete` is metadata only; partial
scans are never discarded. A valid zero-cluster scan removes the previous
marker without using `DELETEALL`. Here, `complete` means that the number of
received General frames equals `expected_near_count + expected_far_count`; it
does not deduplicate IDs or alter `clusters`.

`ClusterList.header.stamp` and the Marker timestamp use the Linux SocketCAN
kernel receive time of the Status frame that began the scan. Each `Cluster`
also contains `rx_stamp`, the kernel receive time of its General frame.
`first_rx_stamp` and `last_rx_stamp` describe the received CAN interval. These
are host kernel receive timestamps: the CAN payload has no absolute internal
radar acquisition timestamp. `rx_stamp_valid`, `header_stamp_valid`, and
`all_rx_stamps_valid` expose whether those values came from the kernel. If the
kernel timestamp ioctl is unavailable, the driver keeps the frame, uses a
local system-clock fallback, sets the validity flag false, and emits a
throttled warning.

The basic receive path intentionally handles Cluster Status and General frames.
Keep `radarcfg_send_quality: 0`; Quality fields remain invalid and are marked by
`quality_valid: false`.

Manual configuration services are node-local. Use `radar_can0` for sensor IDs
`0` and `1`, and `radar_can1` for sensor ID `2`:

```shell
ros2 service call /radar_can0/set_radar_configuration radar_conti_srr308_msgs/srv/TriggerSetCfg "{sensor_id: 0}"
ros2 service call /radar_can0/set_radar_configuration radar_conti_srr308_msgs/srv/TriggerSetCfg "{sensor_id: 1}"
ros2 service call /radar_can1/set_radar_configuration radar_conti_srr308_msgs/srv/TriggerSetCfg "{sensor_id: 2}"
```

`set_radar_configuration` sends only the radar's base-ID `0x200`
RadarConfiguration (`0x210`/`0x220` for sensor IDs 1/2). A parameter ending
in `_valid` must be `1` when that field is intended to change; `0` means that
the radar must ignore the field. For example, disabling extended Object
information requires both `radarcfg_send_ext_info: 0` and
`radarcfg_send_ext_info_valid: 1`. Its service success means SocketCAN accepted
the frame; it does not wait for radar state confirmation. Verify the resulting
`sendextinfocfg` on `/<radar_link>/radar_state`.

Filter configuration remains manual when `enable_can_tx: false`. Apply all
supported SRR308 Cluster filters from the selected radar's YAML block with:

```shell
ros2 service call /radar_can0/set_filter_configuration radar_conti_srr308_msgs/srv/TriggerSetCfg "{sensor_id: 0}"
ros2 service call /radar_can0/set_filter_configuration radar_conti_srr308_msgs/srv/TriggerSetCfg "{sensor_id: 1}"
ros2 service call /radar_can1/set_filter_configuration radar_conti_srr308_msgs/srv/TriggerSetCfg "{sensor_id: 2}"
```

All current YAML `active` entries are false, so these batch calls disable the
four supported Cluster filters. Inactive entries are sent with `Active=0`; their YAML
limits are intentionally ignored and encoded as zero. This removes
`/<radar_link>/fov_filter` but does not affect the static
`/<radar_link>/fov`.

The batch service sends and confirms the four zero-based filters defined for
Cluster type by the reused Continental protocol: `0,1,2,5`. Indices `6..10`
are Object-only; SRR308-unsupported/non-Cluster indices
`3,4,11,12,13,14` are also skipped. Frames use the DBC-required DLC of 5, are
spaced by at least 50 ms, and each must receive a matching base-ID `0x204`
FilterState_Cfg response (`0x214`/`0x224` for sensor IDs 1/2). The service
returns failure if a frame cannot be sent, is rejected/mismatched, or is not
confirmed within 300 ms. All four entries are validated before the first frame
is sent. A later CAN/confirmation failure is not transactional: already
confirmed indices remain applied. A failed index that was sent but not
confirmed may also have been applied; its state is reported as unknown because
the radar has no rollback operation.

Configure one supported filter directly with physical-unit floating-point
limits:

```shell
# Disable the distance filter; limits are ignored when active=false.
ros2 service call /radar_can0/set_filter radar_conti_srr308_msgs/srv/SetFilter \
  "{sensor_id: 0, active: false, type: 0, index: 1, min_value: 0.0, max_value: 0.0}"

# Enable a 0.3 m to 90.0 m distance filter.
ros2 service call /radar_can0/set_filter radar_conti_srr308_msgs/srv/SetFilter \
  "{sensor_id: 0, active: true, type: 0, index: 1, min_value: 0.3, max_value: 90.0}"
```

`type` must be `0` (Cluster). Active limits are checked against the current
DBC encoding before any CAN frame is sent; invalid, non-finite, reversed, or
out-of-range values fail safely instead of wrapping. When `active=false`, the
limits are ignored and encoded as zero. The response reports `sent` separately
from `confirmed`; `success` is true only when both are true.

Desired YAML values and observed FilterState values are stored separately, so
radar replies cannot overwrite the configuration used by a later batch call.
The observed `/<radar_link>/filter_config` message has a `valid_mask`;
bit `i` means that zero-based filter index `i` has actually been received in a
FilterState_Cfg reply. The filtered-FOV marker is drawn only from observed
distance and azimuth state, never from unconfirmed YAML defaults.
Parameters are loaded during lifecycle configure. After editing the YAML file,
restart the launch or run a complete deactivate/cleanup/configure/activate
sequence before triggering the configuration service.

`SetFilter.srv` and `FilterStateCfg.msg` changed in this revision. Rebuild and
source every external ROS 2 client that was compiled against the old
interfaces, and restart the radar nodes before calling `set_filter`.

Use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` in every ROS 2
container that must exchange topics.
