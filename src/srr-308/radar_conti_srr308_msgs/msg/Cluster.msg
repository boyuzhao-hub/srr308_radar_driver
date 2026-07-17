std_msgs/Int32 cluster_id
std_msgs/Int32 sensor_id
# Linux SocketCAN kernel receive time of this Cluster_1_General frame.
# The SRR308 CAN payload does not contain an absolute radar acquisition time,
# so this is the earliest absolute timestamp available to the driver.
builtin_interfaces/Time rx_stamp
# False only when the platform could not provide the kernel CAN timestamp and
# the adapter had to use a local system-clock fallback.
bool rx_stamp_valid
ClusterGeneral cluster_general
bool quality_valid
ClusterQuality cluster_quality
