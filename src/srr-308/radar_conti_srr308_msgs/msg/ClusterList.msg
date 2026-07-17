std_msgs/Header header
std_msgs/Int32 cluster_count

# Source and scan metadata. header.stamp is the SocketCAN kernel receive time
# of Cluster_0_Status, or the first General frame for a status-less scan.
string can_channel
uint8 sensor_id
uint16 measurement_counter
uint8 interface_version
uint16 expected_near_count
uint16 expected_far_count
uint32 received_count
uint32 unique_cluster_count
uint32 duplicate_count
bool status_valid
bool complete
# header_stamp_valid describes header.stamp. all_rx_stamps_valid is true only
# when the Status/General frame timestamps in this list all came from SocketCAN.
bool header_stamp_valid
bool all_rx_stamps_valid
builtin_interfaces/Time first_rx_stamp
builtin_interfaces/Time last_rx_stamp

# General frames in CAN receive order. Duplicate cluster IDs are retained.
Cluster[] clusters
