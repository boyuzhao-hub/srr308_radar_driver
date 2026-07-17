#include "../include/radar_conti_srr308_component.hpp"
#include "../include/offsets.hpp"
#include "../include/visualization.hpp"

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>
#include <math.h>
#include <fmt/core.h>
#include <tf2/LinearMath/Quaternion.h>
#include "../include/radar_transforms.hpp"

#define _USE_MATH_DEFINES

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace FHAC
{

  constexpr std::size_t kConfiguredFilterCount = 15U;
  // The reused Continental FilterCfg protocol defines only these indices for
  // Cluster type. Lifetime, size, probability, X and Y (6..10) are Object
  // filters and must not be sent with Type=Cluster.
  constexpr std::array<uint8_t, 4> kSupportedClusterFilterIndices{
      0U, 1U, 2U, 5U};

  const std::vector<FilterType> filterTypes = {
      FilterType::NOFOBJ,
      FilterType::DISTANCE,
      FilterType::AZIMUTH,
      FilterType::VRELONCOME,
      FilterType::VRELDEPART,
      FilterType::RCS,
      FilterType::LIFETIME,
      FilterType::SIZE,
      FilterType::PROBEXISTS,
      FilterType::Y,
      FilterType::X,
      FilterType::VYRIGHTLEFT,
      FilterType::VXONCOME,
      FilterType::VYLEFTRIGHT,
      FilterType::VXDEPART,
      FilterType::UNKNOWN // Add this to handle default case
  };

  struct FilterValues
  {
    bool active{false};
    double min_value{0.0};
    double max_value{0.0};
  };

  struct FilterRange
  {
    double min_value;
    double max_value;
    double resolution;
    double offset;
    bool integral;
  };

  struct FilterEncoding
  {
    FilterType filter_type{FilterType::UNKNOWN};
    long raw_min{0};
    long raw_max{0};
    double expected_min{0.0};
    double expected_max{0.0};
  };

  bool is_supported_cluster_filter(const uint8_t index)
  {
    return std::find(
               kSupportedClusterFilterIndices.begin(),
               kSupportedClusterFilterIndices.end(),
               index) != kSupportedClusterFilterIndices.end();
  }

  FilterValues get_filter_values(
      const radar_conti_srr308_msgs::msg::FilterStateCfg &config,
      const FilterType filter_type)
  {
    switch (filter_type)
    {
      case FilterType::NOFOBJ:
        return {config.nofobj.active, static_cast<double>(config.nofobj.min), static_cast<double>(config.nofobj.max)};
      case FilterType::DISTANCE:
        return {config.distance.active, config.distance.min, config.distance.max};
      case FilterType::AZIMUTH:
        return {config.azimuth.active, config.azimuth.min, config.azimuth.max};
      case FilterType::VRELONCOME:
        return {config.vreloncome.active, config.vreloncome.min, config.vreloncome.max};
      case FilterType::VRELDEPART:
        return {config.vreldepart.active, config.vreldepart.min, config.vreldepart.max};
      case FilterType::RCS:
        return {config.rcs.active, config.rcs.min, config.rcs.max};
      case FilterType::LIFETIME:
        return {config.lifetime.active, config.lifetime.min, config.lifetime.max};
      case FilterType::SIZE:
        return {config.size.active, config.size.min, config.size.max};
      case FilterType::PROBEXISTS:
        return {config.probexists.active, static_cast<double>(config.probexists.min), static_cast<double>(config.probexists.max)};
      case FilterType::Y:
        return {config.y.active, config.y.min, config.y.max};
      case FilterType::X:
        return {config.x.active, config.x.min, config.x.max};
      case FilterType::VYRIGHTLEFT:
        return {config.vyrightleft.active, config.vyrightleft.min, config.vyrightleft.max};
      case FilterType::VXONCOME:
        return {config.vxoncome.active, config.vxoncome.min, config.vxoncome.max};
      case FilterType::VYLEFTRIGHT:
        return {config.vyleftright.active, config.vyleftright.min, config.vyleftright.max};
      case FilterType::VXDEPART:
        return {config.vxdepart.active, config.vxdepart.min, config.vxdepart.max};
      default:
        return {};
    }
  }

  FilterRange get_filter_range(const FilterType filter_type)
  {
    switch (filter_type)
    {
      case FilterType::NOFOBJ:
        return {0.0, 4095.0, 1.0, 0.0, true};
      case FilterType::DISTANCE:
        return {0.0, 409.5, FilterConfig::DISTANCE_RESOLUTION, 0.0, false};
      case FilterType::AZIMUTH:
        return {-50.0, 52.375, FilterConfig::AZIMUTH_RESOLUTION, FilterConfig::AZIMUTH_OFFSET, false};
      case FilterType::VRELONCOME:
        return {0.0, 128.9925, FilterConfig::VRELONCOME_RESOLUTION, 0.0, false};
      case FilterType::VRELDEPART:
        return {0.0, 128.9925, FilterConfig::VRELDEPART_RESOLUTION, 0.0, false};
      case FilterType::RCS:
        return {-50.0, 52.375, FilterConfig::RCS_RESOLUTION, FilterConfig::RCS_OFFSET, false};
      case FilterType::LIFETIME:
        return {0.0, 409.5, FilterConfig::LIFETIME_RESOLUTION, 0.0, false};
      case FilterType::SIZE:
        return {0.0, 102.375, FilterConfig::SIZE_RESOLUTION, 0.0, false};
      case FilterType::PROBEXISTS:
        return {0.0, 7.0, 1.0, 0.0, true};
      case FilterType::Y:
        return {-409.5, 409.5, FilterConfig::Y_RESOLUTION, FilterConfig::Y_OFFSET, false};
      case FilterType::X:
        return {-500.0, 1138.2, FilterConfig::X_RESOLUTION, FilterConfig::X_OFFSET, false};
      case FilterType::VYRIGHTLEFT:
        return {0.0, 128.9925, FilterConfig::VYRIGHTLEFT_RESOLUTION, 0.0, false};
      case FilterType::VXONCOME:
        return {0.0, 128.9925, FilterConfig::VXONCOME_RESOLUTION, 0.0, false};
      case FilterType::VYLEFTRIGHT:
        return {0.0, 128.9925, FilterConfig::VYLEFTRIGHT_RESOLUTION, 0.0, false};
      case FilterType::VXDEPART:
        return {0.0, 128.9925, FilterConfig::VXDEPART_RESOLUTION, 0.0, false};
      default:
        return {0.0, 0.0, 1.0, 0.0, true};
    }
  }

  bool validate_filter_request(
      const bool active, const uint8_t type, const uint8_t index,
      const double min_value, const double max_value,
      FilterEncoding &encoding, std::string &message)
  {
    encoding = FilterEncoding{};
    if (type != FilterCfg_FilterCfg_Type_Cluster)
    {
      message = fmt::format(
          "Filter type {} is unsupported; this basic driver accepts Cluster type {} only",
          type, FilterCfg_FilterCfg_Type_Cluster);
      return false;
    }
    if (index >= kConfiguredFilterCount)
    {
      message = fmt::format(
          "Filter index {} is outside the configured range [0, {}]",
          index, kConfiguredFilterCount - 1U);
      return false;
    }
    if (!is_supported_cluster_filter(index))
    {
      if (index >= 6U && index <= 10U)
      {
        message = fmt::format(
            "Filter index {} is Object-only in the reused Continental "
            "protocol and cannot be sent with Cluster type",
            index);
      }
      else
      {
        message = fmt::format(
            "SRR308 Cluster mode does not support filter index {}", index);
      }
      return false;
    }

    encoding.filter_type = filterTypes[index];
    if (!active)
    {
      // Inactive FilterCfg frames intentionally encode both limits as zero;
      // the radar ignores them when Active=0.
      return true;
    }

    if (!std::isfinite(min_value) || !std::isfinite(max_value))
    {
      message = "Filter limits must be finite";
      return false;
    }
    if (min_value > max_value)
    {
      message = fmt::format(
          "Filter minimum {} is greater than maximum {}", min_value, max_value);
      return false;
    }

    const auto range = get_filter_range(encoding.filter_type);
    const double range_epsilon = std::max(1e-9, range.resolution * 1e-6);
    if (min_value < range.min_value - range_epsilon ||
        max_value > range.max_value + range_epsilon)
    {
      message = fmt::format(
          "Filter index {} limits [{}, {}] are outside the encodable range [{}, {}]",
          index, min_value, max_value, range.min_value, range.max_value);
      return false;
    }
    if (range.integral &&
        (std::abs(min_value - std::round(min_value)) > range_epsilon ||
         std::abs(max_value - std::round(max_value)) > range_epsilon))
    {
      message = fmt::format(
          "Filter index {} requires integer-valued limits", index);
      return false;
    }

    encoding.raw_min =
        std::lround((min_value + range.offset) / range.resolution);
    encoding.raw_max =
        std::lround((max_value + range.offset) / range.resolution);
    const long lowest_raw =
        std::lround((range.min_value + range.offset) / range.resolution);
    const long highest_raw =
        std::lround((range.max_value + range.offset) / range.resolution);
    if (encoding.raw_min < lowest_raw || encoding.raw_min > highest_raw ||
        encoding.raw_max < lowest_raw || encoding.raw_max > highest_raw)
    {
      message = fmt::format(
          "Filter index {} quantized limits [{}, {}] exceed raw range [{}, {}]",
          index, encoding.raw_min, encoding.raw_max, lowest_raw, highest_raw);
      return false;
    }

    encoding.expected_min =
        static_cast<double>(encoding.raw_min) * range.resolution - range.offset;
    encoding.expected_max =
        static_cast<double>(encoding.raw_max) * range.resolution - range.offset;
    return true;
  }

  radar_conti_srr308::radar_conti_srr308(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("radar_conti_srr308", options)
  {
  }

  radar_conti_srr308::~radar_conti_srr308()
  {
    // The SocketCAN adapter is declared before the per-radar buffers and is
    // therefore destroyed after them by default. Stop its callback thread in
    // the destructor body, while every object touched by can_receive_callback
    // is still alive. This path also covers SIGINT, where ROS can invalidate
    // the context without first executing the lifecycle deactivate callback.
    {
      std::lock_guard<std::mutex> confirmation_lock(filter_confirmation_mutex_);
      if (pending_filter_confirmation_.waiting)
      {
        pending_filter_confirmation_.received = true;
        pending_filter_confirmation_.matched = false;
        pending_filter_confirmation_.cancelled = true;
      }
    }
    filter_confirmation_cv_.notify_all();

    if (socketcan_adapter_)
    {
      socketcan_adapter_->joinReceptionThread(std::chrono::duration<float>(0.0F));
      if (socketcan_adapter_->get_socket_state() != polymath::socketcan::SocketState::CLOSED)
      {
        socketcan_adapter_->closeSocket();
      }
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_srr308::on_configure(
      const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    auto node = shared_from_this();
    const auto declare_if_needed =
        [&node](const std::string &name, const rclcpp::ParameterValue &value)
        {
          if (!node->has_parameter(name))
          {
            node->declare_parameter(name, value);
          }
        };

    declare_if_needed("can_channel", rclcpp::ParameterValue(""));
    declare_if_needed("odom_topic_name", rclcpp::ParameterValue(""));
    declare_if_needed("robot_base_frame", rclcpp::ParameterValue("base_link"));
    declare_if_needed("transform_timeout", rclcpp::ParameterValue(0.1));
    declare_if_needed("object_list_topic_name", rclcpp::ParameterValue("srr308/object_list"));
    // SRR308 Change: Add cluster list topic
    declare_if_needed("cluster_list_topic_name", rclcpp::ParameterValue("srr308/cluster_list"));

    declare_if_needed("marker_array_topic_name", rclcpp::ParameterValue("srr308/marker_array"));
    declare_if_needed("radar_tracks_topic_name", rclcpp::ParameterValue("srr308/radar_tracks"));
    declare_if_needed("obstacle_array_topic_name", rclcpp::ParameterValue("srr308/obstacle_array"));
    declare_if_needed("filter_config_topic_name", rclcpp::ParameterValue("srr308/filter_config"));
    declare_if_needed("radar_state_topic_name", rclcpp::ParameterValue("srr308/radar_state"));
    declare_if_needed("qos_deadline_hz", rclcpp::ParameterValue(10));
    declare_if_needed("enable_can_tx", rclcpp::ParameterValue(false));
    declare_if_needed("enable_manual_can_tx", rclcpp::ParameterValue(true));
    declare_if_needed("cluster_marker_lifetime_sec", rclcpp::ParameterValue(2.0));
    declare_if_needed("cluster_marker_scale", rclcpp::ParameterValue(0.8));
    declare_if_needed("cluster_scan_timeout_ms", rclcpp::ParameterValue(200));
    declare_if_needed("max_cluster_frames_per_scan", rclcpp::ParameterValue(512));

    double transform_timeout_double;
    int qos_deadline_hz;
    int cluster_scan_timeout_ms;
    int max_cluster_frames_per_scan;
    node->get_parameter("can_channel", can_channel_);
    node->get_parameter("odom_topic_name", odom_topic_name_);
    node->get_parameter("robot_base_frame", robot_base_frame_);
    node->get_parameter("transform_timeout", transform_timeout_double);
    node->get_parameter("object_list_topic_name", object_list_topic_name_);
    // SRR308 Change: Add cluster list topic
    node->get_parameter("cluster_list_topic_name", cluster_list_topic_name_);

    node->get_parameter("marker_array_topic_name", marker_array_topic_name_);
    node->get_parameter("radar_tracks_topic_name", radar_tracks_topic_name_);
    node->get_parameter("obstacle_array_topic_name", obstacle_array_topic_name_);
    node->get_parameter("filter_config_topic_name", filter_config_topic_name_);
    node->get_parameter("radar_state_topic_name", radar_state_topic_name_);
    node->get_parameter("qos_deadline_hz", qos_deadline_hz);
    node->get_parameter("enable_can_tx", enable_can_tx_);
    node->get_parameter("enable_manual_can_tx", enable_manual_can_tx_);
    node->get_parameter("cluster_marker_lifetime_sec", cluster_marker_lifetime_sec_);
    node->get_parameter("cluster_marker_scale", cluster_marker_scale_);
    node->get_parameter("cluster_scan_timeout_ms", cluster_scan_timeout_ms);
    node->get_parameter("max_cluster_frames_per_scan", max_cluster_frames_per_scan);

    if (qos_deadline_hz <= 0)
    {
      RCLCPP_WARN(node->get_logger(), "qos_deadline_hz must be positive; using 10 Hz");
      qos_deadline_hz = 10;
    }
    if (cluster_marker_lifetime_sec_ <= 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "cluster_marker_lifetime_sec must be positive; using 2.0 seconds");
      cluster_marker_lifetime_sec_ = 2.0;
    }
    if (cluster_marker_scale_ <= 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "cluster_marker_scale must be positive; using 0.8 meters");
      cluster_marker_scale_ = 0.8;
    }
    if (cluster_scan_timeout_ms <= 0)
    {
      RCLCPP_WARN(node->get_logger(), "cluster_scan_timeout_ms must be positive; using 200 ms");
      cluster_scan_timeout_ms = 200;
    }
    cluster_scan_timeout_ = std::chrono::milliseconds(cluster_scan_timeout_ms);
    if (max_cluster_frames_per_scan <= 0)
    {
      RCLCPP_WARN(node->get_logger(), "max_cluster_frames_per_scan must be positive; using 512");
      max_cluster_frames_per_scan = 512;
    }
    max_cluster_frames_per_scan_ =
        static_cast<std::size_t>(max_cluster_frames_per_scan);

    if (can_channel_.empty())
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "No can_channel_ specified.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "Listening on can_channel %s", can_channel_.c_str());
    RCUTILS_LOG_INFO_NAMED(get_name(), "Automatic CAN transmit is %s", enable_can_tx_ ? "enabled" : "disabled");
    RCUTILS_LOG_INFO_NAMED(get_name(), "Manual CAN service transmit is %s", enable_manual_can_tx_ ? "enabled" : "disabled");

    auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto filter_state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto static_fov_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);

    rclcpp::Duration deadline{std::chrono::seconds(1) / static_cast<float>(qos_deadline_hz)};
    auto radar_tracks_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile).deadline(deadline);

    size_t topic_ind = 0;
    bool more_params = false;
    number_of_radars_ = 0;
    std::unordered_set<std::string> configured_link_names;
    do
    {
      // Build the string in the form of "radar_link_X", where X is the sensor ID of
      // the rader on the CANBUS, then check if we have any parameters with that value. Users need
      // to make sure they don't have gaps in their configs (e.g.,footprint0 and then
      // footprint2)
      std::stringstream ss;
      ss << "radar_" << topic_ind;
      std::string radar_name = ss.str();

      if (!node->has_parameter(radar_name + ".link_name"))
      {
        node->declare_parameter(radar_name + ".link_name", rclcpp::PARAMETER_STRING);
      }

      rclcpp::Parameter parameter;
      if (node->get_parameter(radar_name + ".link_name", parameter))
      {
        more_params = true;
        if (parameter.as_string().empty())
        {
          RCLCPP_ERROR(node->get_logger(), "%s.link_name must not be empty", radar_name.c_str());
          return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        if (!configured_link_names.insert(parameter.as_string()).second)
        {
          RCLCPP_ERROR(
              node->get_logger(), "Duplicate radar link_name '%s' on %s",
              parameter.as_string().c_str(), can_channel_.c_str());
          return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        object_list_publishers_.push_back(this->create_publisher<radar_conti_srr308_msgs::msg::ObjectList>(parameter.as_string() + "/" + object_list_topic_name_, qos));
        tf_publishers_.push_back(this->create_publisher<tf2_msgs::msg::TFMessage>(parameter.as_string() + "/" + pub_tf_topic_name, qos));
        marker_array_publishers_.push_back(this->create_publisher<visualization_msgs::msg::MarkerArray>(parameter.as_string() + "/" + marker_array_topic_name_, qos));
        fov_marker_publishers_.push_back(this->create_publisher<visualization_msgs::msg::MarkerArray>(parameter.as_string() + "/fov", static_fov_qos));
        fov_filter_marker_publishers_.push_back(this->create_publisher<visualization_msgs::msg::Marker>(parameter.as_string() + "/fov_filter", qos));
        radar_tracks_publishers_.push_back(this->create_publisher<radar_msgs::msg::RadarTracks>(parameter.as_string() + "/" + radar_tracks_topic_name_, radar_tracks_qos));
        obstacle_array_publishers_.push_back(this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>(parameter.as_string() + "/" + obstacle_array_topic_name_, radar_tracks_qos));
        filter_config_publishers_.push_back(this->create_publisher<radar_conti_srr308_msgs::msg::FilterStateCfg>(parameter.as_string() + "/" + filter_config_topic_name_, filter_state_qos));
        radar_state_publishers_.push_back(this->create_publisher<radar_conti_srr308_msgs::msg::RadarState>(parameter.as_string() + "/" + radar_state_topic_name_, transient_local_qos));
        object_map_list_.push_back(std::map<int, radar_conti_srr308_msgs::msg::Object>());
        // SRR308 Changes: add cluter mode
        cluster_list_publishers_.push_back(this->create_publisher<radar_conti_srr308_msgs::msg::ClusterList>(parameter.as_string() + "/" + cluster_list_topic_name_, qos));
        cluster_scans_.emplace_back();
        // ##############################################################
        object_list_list_.push_back(radar_conti_srr308_msgs::msg::ObjectList());
        desired_filter_configs_.emplace_back();
        radar_filter_configs_.emplace_back();
        radar_configuration_configs_.emplace_back();
        radar_filter_active_.push_back(std::vector<bool>());

        std::vector<bool> init_radar_active_values(kConfiguredFilterCount, false);
        declare_if_needed(radar_name + ".active", rclcpp::ParameterValue(init_radar_active_values));
        node->get_parameter(radar_name + ".active", radar_filter_active_[topic_ind]);
        if (radar_filter_active_[topic_ind].size() != kConfiguredFilterCount)
        {
          RCLCPP_ERROR(
              node->get_logger(), "%s.active must contain exactly %zu entries",
              radar_name.c_str(), kConfiguredFilterCount);
          return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        for (std::size_t filter_index = 0; filter_index < kConfiguredFilterCount; ++filter_index)
        {
          if (radar_filter_active_[topic_ind][filter_index] &&
              !is_supported_cluster_filter(static_cast<uint8_t>(filter_index)))
          {
            RCLCPP_ERROR(
                node->get_logger(),
                "%s.active[%zu] is true, but SRR308 Cluster mode does not support this filter index",
                radar_name.c_str(), filter_index);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
          }
        }

        // Send motion state
        declare_if_needed(radar_name + ".send_motion", rclcpp::ParameterValue(false));
        node->get_parameter(radar_name + ".send_motion", motion_configs_[topic_ind]);

        radar_link_names_.push_back(parameter.as_string());

        RCLCPP_DEBUG(node->get_logger(), "radar frame is: %s", parameter.as_string().c_str());

        // RADAR CONFIGS

        // NVM Storage
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_store_in_nvm"), 0, radar_configuration_configs_[topic_ind].radarcfg_storeinnvm.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_store_in_nvm_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_storeinnvm_valid.data);

        // Ext Info
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_ext_info"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendextinfo.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_ext_info_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendextinfo_valid.data);

        // Ctrl Relay
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_ctrl_relay"), 0, radar_configuration_configs_[topic_ind].radarcfg_ctrlrelay.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_ctrl_relay_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_ctrlrelay_valid.data);

        // Radar Power
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_radar_power"), START_RadarConfiguration_RadarCfg_RadarPower, radar_configuration_configs_[topic_ind].radarcfg_radarpower.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_radar_power_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_radarpower_valid.data);

        // Send Quality
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_quality"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendquality.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_quality_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendquality_valid.data);
        if (radar_configuration_configs_[topic_ind].radarcfg_sendquality.data != 0)
        {
          RCLCPP_WARN(
              node->get_logger(),
              "%s requests Quality output, but this basic driver publishes General frames only",
              radar_name.c_str());
        }

        // Max Distance
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_max_distance"), 0, radar_configuration_configs_[topic_ind].radarcfg_maxdistance.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_max_distance_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_maxdistance_valid.data);

        // Sensor ID
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_sensor_id"), 0, radar_configuration_configs_[topic_ind].radarcfg_sensorid.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_sensor_id_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_sensorid_valid.data);
        const int configured_sensor_id = radar_configuration_configs_[topic_ind].radarcfg_sensorid.data;
        if (configured_sensor_id < 0 || configured_sensor_id > 7)
        {
          RCLCPP_ERROR(
              node->get_logger(), "%s.radarcfg_sensor_id=%d is invalid; expected [0, 7]",
              radar_name.c_str(), configured_sensor_id);
          return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        if (!sensor_id_to_local_index_.emplace(configured_sensor_id, topic_ind).second)
        {
          RCLCPP_ERROR(
              node->get_logger(), "Duplicate radar sensor ID %d on CAN channel %s",
              configured_sensor_id, can_channel_.c_str());
          return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        radar_sensor_ids_.push_back(configured_sensor_id);

        // RCS Threshold
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_rcs_threshold"), 0, radar_configuration_configs_[topic_ind].radarcfg_rcs_threshold.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_rcs_threshold_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_rcs_threshold_valid.data);

        // Output Type
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_output_type"), 0, radar_configuration_configs_[topic_ind].radarcfg_outputtype.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_output_type_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_outputtype_valid.data);
        if (radar_configuration_configs_[topic_ind].radarcfg_outputtype.data ==
            RadarState_RadarState_OutputTypeCfg_SendObjects)
        {
          RCLCPP_ERROR(
              node->get_logger(),
              "%s is configured for Object output; this receive path is intentionally Cluster-only",
              radar_name.c_str());
          return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        // Sort Index
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_sort_index"), 0, radar_configuration_configs_[topic_ind].radarcfg_sortindex.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_sort_index_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_sortindex_valid.data);

        // FILTER CONFIGS
        // Initialize Number of Objects Filters
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_min_nofobj"), 0, desired_filter_configs_[topic_ind].nofobj.min);
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_max_nofobj"), 20, desired_filter_configs_[topic_ind].nofobj.max);

        // Initialize Distance Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_distance"), 0.0, desired_filter_configs_[topic_ind].distance.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_distance"), 409.0, desired_filter_configs_[topic_ind].distance.max);

        // Initialize Azimuth Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_azimuth"), -50.0, desired_filter_configs_[topic_ind].azimuth.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_azimuth"), 50.0, desired_filter_configs_[topic_ind].azimuth.max);

        // Initialize Oncoming Velocity Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vreloncome"), 0.0, desired_filter_configs_[topic_ind].vreloncome.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vreloncome"), 128.0, desired_filter_configs_[topic_ind].vreloncome.max);

        // Initialize Departing Velocity Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vreldepart"), 0.0, desired_filter_configs_[topic_ind].vreldepart.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vreldepart"), 128.0, desired_filter_configs_[topic_ind].vreldepart.max);

        // Initialize Radar Cross Section Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_rcs"), -20.0, desired_filter_configs_[topic_ind].rcs.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_rcs"), 30.0, desired_filter_configs_[topic_ind].rcs.max);

        // Initialize Lifetime Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_lifetime"), 0.0, desired_filter_configs_[topic_ind].lifetime.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_lifetime"), 409.0, desired_filter_configs_[topic_ind].lifetime.max);

        // Initialize Size Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_size"), 0.0, desired_filter_configs_[topic_ind].size.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_size"), 102.0, desired_filter_configs_[topic_ind].size.max);

        // Initialize Probability of Existence Filters
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_min_probexists"), 0, desired_filter_configs_[topic_ind].probexists.min);
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_max_probexists"), 7, desired_filter_configs_[topic_ind].probexists.max);

        // Initialize Y Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_y"), -400.0, desired_filter_configs_[topic_ind].y.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_y"), 400.0, desired_filter_configs_[topic_ind].y.max);

        // Initialize X Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_x"), 0.0, desired_filter_configs_[topic_ind].x.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_x"), 0.0, desired_filter_configs_[topic_ind].x.max);

        // Initialize Y Velocity going left to right Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vyrightleft"), 0.0, desired_filter_configs_[topic_ind].vyrightleft.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vyrightleft"), 128.0, desired_filter_configs_[topic_ind].vyrightleft.max);

        // Initialize X Velocity oncoming Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vxoncome"), 0.0, desired_filter_configs_[topic_ind].vxoncome.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vxoncome"), 128.0, desired_filter_configs_[topic_ind].vxoncome.max);

        // Initialize Y Velocity going right to left Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vyleftright"), 0.0, desired_filter_configs_[topic_ind].vyleftright.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vyleftright"), 128.0, desired_filter_configs_[topic_ind].vyleftright.max);

        // Initialize X Velocity Departing Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vxdepart"), 0.0, desired_filter_configs_[topic_ind].vxdepart.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vxdepart"), 128.0, desired_filter_configs_[topic_ind].vxdepart.max);

        RCLCPP_WARN(this->get_logger(), "link_name is: %s", parameter.as_string().c_str());
        number_of_radars_++;
        topic_ind++;
      }
      else
      {
        more_params = false;
      }
    } while (more_params);

    if (number_of_radars_ == 0)
    {
      RCLCPP_ERROR(node->get_logger(), "No radar_N configuration found for %s", can_channel_.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // TODO(troy): Make a user configurable recv_timeout
    constexpr std::chrono::duration<float> recv_timeout{0.1};
    socketcan_adapter_ = std::make_unique<polymath::socketcan::SocketcanAdapter>(can_channel_, recv_timeout);
    object_count = 0.0;
    set_filter_service_ = create_service<radar_conti_srr308_msgs::srv::SetFilter>("~/set_filter", std::bind(&radar_conti_srr308::setFilterService, this, std::placeholders::_1, std::placeholders::_2));
    filter_config_service_ = create_service<radar_conti_srr308_msgs::srv::TriggerSetCfg>("~/set_filter_configuration", std::bind(&radar_conti_srr308::setFilterConfigurationService, this, std::placeholders::_1, std::placeholders::_2));
    radar_config_service_ = create_service<radar_conti_srr308_msgs::srv::TriggerSetCfg>("~/set_radar_configuration", std::bind(&radar_conti_srr308::setRadarConfigurationService, this, std::placeholders::_1, std::placeholders::_2));
    if (!odom_topic_name_.empty())
    {
      rclcpp::QoS qos_settings(10);
      qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_name_, qos_settings, std::bind(&radar_conti_srr308::odomCallback, this, std::placeholders::_1));
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    transform_timeout_ = rclcpp::Duration::from_seconds(transform_timeout_double);
    generateUUIDTable();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  template <typename T>
  void radar_conti_srr308::initializeConfig(std::string radar_name, std::string config_name, T default_value, T &config)
  {
    auto node = shared_from_this();
    T config_value;
    declare_parameter_with_type(node, radar_name + "." + config_name, default_value);
    get_parameter_with_type(node, radar_name + "." + config_name, config_value);
    config = config_value;
  }

  rclcpp::Time radar_conti_srr308::frame_bus_time(
      const polymath::socketcan::CanFrame &frame) const
  {
    // SocketcanAdapter stores the Linux kernel receive timestamp as a
    // system_clock time point. The CAN payload has no absolute acquisition
    // time, so this is the earliest absolute time available to this driver.
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        frame.get_bus_time().time_since_epoch()).count();
    return rclcpp::Time(static_cast<int64_t>(nanoseconds), RCL_SYSTEM_TIME);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_srr308::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called.");
    if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      return on_deactivate(previous_state);
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_srr308::on_error(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on error is called.");
    if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      const auto result = on_deactivate(previous_state);
      if (result != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
      {
        return result;
      }
    }
    return on_cleanup(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_srr308::on_activate(
      const rclcpp_lifecycle::State &)
  {
    // A new SocketCAN receive session has no confirmed FilterState yet. Do
    // not carry observations across a CAN reconnect or possible radar reboot.
    resetFilterObservations();

    if (!socketcan_adapter_->openSocket())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open socket on can channel '%s'", can_channel_.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // The middle byte is used by SRR308 as the sensor id, so we just clear it out here.
    canid_t id_mask = 0xF0F;
    // can_filter radar_obj_filter{
    //     ID_Obj_1_General,
    //     id_mask,
    // };

    can_filter filter_state_filter{
        ID_FilterState_Cfg,
        id_mask,
    };

    can_filter radar_state_filter{
        ID_RadarState,
        id_mask,
    };

    // can_filter radar_obj_status_filter{
    //     ID_Obj_0_Status,
    //     id_mask,
    // };

    // can_filter radar_obj_quality_filter{
    //     ID_Obj_2_Quality,
    //     id_mask,
    // };

    // can_filter radar_obj_extended{
    //     ID_Obj_3_Extended,
    //     id_mask,
    // };

    // SRR308 Change: Add cluster mode
    can_filter radar_cluster_status_filter{
        ID_Cluster_0_Status,
        id_mask,
    };

    can_filter radar_cluster_filter{
        ID_Cluster_1_General,
        id_mask,
    };

    const auto filter_error = socketcan_adapter_->setFilters(std::vector<can_filter>{
        filter_state_filter,
        radar_state_filter,
        radar_cluster_status_filter,
        radar_cluster_filter,
    });
    if (filter_error.has_value())
    {
      RCLCPP_ERROR(
          this->get_logger(), "Unable to install CAN filters on '%s': %s",
          can_channel_.c_str(), filter_error->c_str());
      socketcan_adapter_->closeSocket();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    auto cb = [this](std::unique_ptr<const polymath::socketcan::CanFrame> frame)
    {
      std::shared_ptr<const polymath::socketcan::CanFrame> shared_frame = std::move(frame);
      this->can_receive_callback(shared_frame);
    };

    socketcan_adapter_->setOnReceiveCallback(std::move(cb));

    for (size_t i = 0; i < object_list_publishers_.size(); i++)
    {
      object_list_publishers_[i]->on_activate();
      // SRR308 Change: add cluster mode
      cluster_list_publishers_[i]->on_activate();

      tf_publishers_[i]->on_activate();
      marker_array_publishers_[i]->on_activate();
      fov_marker_publishers_[i]->on_activate();
      fov_filter_marker_publishers_[i]->on_activate();
      radar_tracks_publishers_[i]->on_activate();
      obstacle_array_publishers_[i]->on_activate();
      filter_config_publishers_[i]->on_activate();
      radar_state_publishers_[i]->on_activate();
    }
    publishFilteredFovDeleteMarkers();
    publishClearedFilterObservations();

    if (!socketcan_adapter_->startReceptionThread())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to start CAN receive thread on '%s'", can_channel_.c_str());
      for (size_t i = 0; i < object_list_publishers_.size(); i++)
      {
        object_list_publishers_[i]->on_deactivate();
        cluster_list_publishers_[i]->on_deactivate();
        tf_publishers_[i]->on_deactivate();
        marker_array_publishers_[i]->on_deactivate();
        fov_marker_publishers_[i]->on_deactivate();
        fov_filter_marker_publishers_[i]->on_deactivate();
        radar_tracks_publishers_[i]->on_deactivate();
        obstacle_array_publishers_[i]->on_deactivate();
        filter_config_publishers_[i]->on_deactivate();
        radar_state_publishers_[i]->on_deactivate();
      }
      socketcan_adapter_->closeSocket();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    const auto timeout_check_period =
        std::max(std::chrono::milliseconds(10), cluster_scan_timeout_ / 2);
    cluster_scan_timeout_timer_ = this->create_wall_timer(
        timeout_check_period,
        std::bind(&radar_conti_srr308::check_cluster_scan_timeouts, this));

    // The static SRR308 field of view does not depend on FilterState. Publish
    // it immediately after the lifecycle publishers are active, then retain
    // the periodic timer for volatile RViz subscribers that join later.
    publishFovMetadata();
    startMetadataTimers();

    if (enable_can_tx_)
    {
      initializeFilterConfigs();
    }
    else
    {
      RCLCPP_WARN(
          this->get_logger(),
          "Automatic CAN transmit is disabled; skipping startup filter configuration writes.");
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_srr308::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    {
      std::lock_guard<std::mutex> confirmation_lock(filter_confirmation_mutex_);
      if (pending_filter_confirmation_.waiting)
      {
        pending_filter_confirmation_.received = true;
        pending_filter_confirmation_.matched = false;
        pending_filter_confirmation_.cancelled = true;
      }
    }
    filter_confirmation_cv_.notify_all();

    if (filter_config_timer_)
    {
      filter_config_timer_->cancel();
      filter_config_timer_.reset();
    }
    if (fov_marker_timer_)
    {
      fov_marker_timer_->cancel();
      fov_marker_timer_.reset();
    }
    if (cluster_scan_timeout_timer_)
    {
      cluster_scan_timeout_timer_->cancel();
      cluster_scan_timeout_timer_.reset();
    }
    if (socketcan_adapter_)
    {
      socketcan_adapter_->joinReceptionThread(std::chrono::duration<float>(0.0F));
    }

    // No CAN callback can mutate the scan states after the receive thread is
    // joined. Publish every pending scan before deactivating its publishers.
    flush_pending_cluster_scans();
    publishFilteredFovDeleteMarkers();
    resetFilterObservations();
    publishClearedFilterObservations();

    if (socketcan_adapter_ &&
        socketcan_adapter_->get_socket_state() != polymath::socketcan::SocketState::CLOSED &&
        !socketcan_adapter_->closeSocket())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to close socket");
    }

    for (size_t i = 0; i < object_list_publishers_.size(); i++)
    {
      object_list_publishers_[i]->on_deactivate();
      cluster_list_publishers_[i]->on_deactivate();
      tf_publishers_[i]->on_deactivate();
      marker_array_publishers_[i]->on_deactivate();
      fov_marker_publishers_[i]->on_deactivate();
      fov_filter_marker_publishers_[i]->on_deactivate();
      radar_tracks_publishers_[i]->on_deactivate();
      obstacle_array_publishers_[i]->on_deactivate();
      filter_config_publishers_[i]->on_deactivate();
      radar_state_publishers_[i]->on_deactivate();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_srr308::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    if (cluster_scan_timeout_timer_)
    {
      cluster_scan_timeout_timer_->cancel();
      cluster_scan_timeout_timer_.reset();
    }
    if (filter_config_timer_)
    {
      filter_config_timer_->cancel();
      filter_config_timer_.reset();
    }
    if (fov_marker_timer_)
    {
      fov_marker_timer_->cancel();
      fov_marker_timer_.reset();
    }
    if (socketcan_adapter_)
    {
      socketcan_adapter_->joinReceptionThread(std::chrono::duration<float>(0.0F));
      if (socketcan_adapter_->get_socket_state() != polymath::socketcan::SocketState::CLOSED)
      {
        socketcan_adapter_->closeSocket();
      }
      socketcan_adapter_.reset();
    }

    odometry_subscriber_.reset();
    set_filter_service_.reset();
    filter_config_service_.reset();
    radar_config_service_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    object_list_publishers_.clear();
    cluster_list_publishers_.clear();
    tf_publishers_.clear();
    marker_array_publishers_.clear();
    fov_marker_publishers_.clear();
    fov_filter_marker_publishers_.clear();
    radar_tracks_publishers_.clear();
    obstacle_array_publishers_.clear();
    filter_config_publishers_.clear();
    radar_state_publishers_.clear();

    object_map_list_.clear();
    object_list_list_.clear();
    {
      std::lock_guard<std::mutex> lock(cluster_scan_mutex_);
      cluster_scans_.clear();
    }
    desired_filter_configs_.clear();
    radar_filter_configs_.clear();
    radar_configuration_configs_.clear();
    motion_configs_.clear();
    radar_filter_active_.clear();
    radar_sensor_ids_.clear();
    sensor_id_to_local_index_.clear();
    radar_link_names_.clear();
    {
      std::lock_guard<std::mutex> confirmation_lock(filter_confirmation_mutex_);
      pending_filter_confirmation_ = PendingFilterConfirmation{};
    }
    UUID_table_.clear();
    number_of_radars_ = 0;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  unique_identifier_msgs::msg::UUID radar_conti_srr308::generateRandomUUID()
  {
    unique_identifier_msgs::msg::UUID uuid;
    std::mt19937 gen(std::random_device{}());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
    return uuid;
  }

  void radar_conti_srr308::generateUUIDTable()
  {
    for (int i = 0; i <= (max_radar_id * number_of_radars_); i++)
    {
      UUID_table_.emplace_back(radar_conti_srr308::generateRandomUUID());
    }
  }

  void radar_conti_srr308::startMetadataTimers()
  {
    if (!filter_config_timer_)
    {
      filter_config_timer_ = this->create_wall_timer(
          1s, std::bind(&radar_conti_srr308::publishFilterConfigMetadata, this));
    }

    if (!fov_marker_timer_)
    {
      fov_marker_timer_ = this->create_wall_timer(
          3s, std::bind(&radar_conti_srr308::publishFovMetadata, this));
    }
  }

  void radar_conti_srr308::initializeFilterConfigs()
  {
    std::lock_guard<std::mutex> tx_lock(can_tx_mutex_);
    for (std::size_t local_index = 0; local_index < desired_filter_configs_.size(); ++local_index)
    {
      std::string message;
      if (!applyFilterConfiguration(local_index, message))
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "Automatic filter configuration failed for sensor %d: %s",
            radar_sensor_ids_[local_index], message.c_str());
      }
    }
  }

  void radar_conti_srr308::publishRadarState(std::shared_ptr<const polymath::socketcan::CanFrame> frame, const int &sensor_id)
  {
    if (frame->get_len() < DLC_RadarState)
    {
      RCLCPP_WARN(
          this->get_logger(), "Ignoring short RadarState from sensor %d: DLC=%u",
          radar_sensor_ids_[sensor_id], static_cast<unsigned int>(frame->get_len()));
      return;
    }

    radar_conti_srr308_msgs::msg::RadarState radar_state_msg;
    radar_state_msg.header.stamp = frame_bus_time(*frame);
    radar_state_msg.header.frame_id = radar_link_names_[sensor_id];

    radar_state_msg.nvmwritestatus = CALC_RadarState_RadarState_NVMwriteStatus(GET_RadarState_RadarState_NVMwriteStatus(frame->get_data()), 1.0);
    radar_state_msg.nvmreadstatus = CALC_RadarState_RadarState_NVMReadStatus(GET_RadarState_RadarState_NVMReadStatus(frame->get_data()), 1.0);
    radar_state_msg.maxdistancecfg = CALC_RadarState_RadarState_MaxDistanceCfg(GET_RadarState_RadarState_MaxDistanceCfg(frame->get_data()), 1.0);
    radar_state_msg.persistent_error = CALC_RadarState_RadarState_Persistent_Error(GET_RadarState_RadarState_Persistent_Error(frame->get_data()), 1.0);
    radar_state_msg.interference = CALC_RadarState_RadarState_Interference(GET_RadarState_RadarState_Interference(frame->get_data()), 1.0);
    radar_state_msg.temperature_error = CALC_RadarState_RadarState_Temperature_Error(GET_RadarState_RadarState_Temperature_Error(frame->get_data()), 1.0);
    radar_state_msg.temporary_error = CALC_RadarState_RadarState_Temporary_Error(GET_RadarState_RadarState_Temporary_Error(frame->get_data()), 1.0);
    radar_state_msg.voltage_error = CALC_RadarState_RadarState_Voltage_Error(GET_RadarState_RadarState_Voltage_Error(frame->get_data()), 1.0);
    radar_state_msg.radarpowercfg = CALC_RadarState_RadarState_RadarPowerCfg(GET_RadarState_RadarState_RadarPowerCfg(frame->get_data()), 1.0);
    radar_state_msg.sortindex = CALC_RadarState_RadarState_SortIndex(GET_RadarState_RadarState_SortIndex(frame->get_data()), 1.0);
    radar_state_msg.sensorid = CALC_RadarState_RadarState_SensorID(GET_RadarState_RadarState_SensorID(frame->get_data()), 1.0);
    radar_state_msg.motionrxstate = CALC_RadarState_RadarState_MotionRxState(GET_RadarState_RadarState_MotionRxState(frame->get_data()), 1.0);
    radar_state_msg.sendextinfocfg = CALC_RadarState_RadarState_SendExtInfoCfg(GET_RadarState_RadarState_SendExtInfoCfg(frame->get_data()), 1.0);
    radar_state_msg.sendqualitycfg = CALC_RadarState_RadarState_SendQualityCfg(GET_RadarState_RadarState_SendQualityCfg(frame->get_data()), 1.0);
    radar_state_msg.outputtypecfg = CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(frame->get_data()), 1.0);
    radar_state_msg.ctrlrelaycfg = CALC_RadarState_RadarState_CtrlRelayCfg(GET_RadarState_RadarState_CtrlRelayCfg(frame->get_data()), 1.0);
    radar_state_msg.rcs_threshold = CALC_RadarState_RadarState_RCS_Threshold(GET_RadarState_RadarState_RCS_Threshold(frame->get_data()), 1.0);

    if (radar_state_publishers_[sensor_id])
    {
      radar_state_publishers_[sensor_id]->publish(radar_state_msg);
    }
  }

  void radar_conti_srr308::can_receive_callback(std::shared_ptr<const polymath::socketcan::CanFrame> frame)
  {
    if (frame->get_frame_type() != polymath::socketcan::FrameType::DATA ||
        frame->get_id_type() != polymath::socketcan::IdType::STANDARD)
    {
      return;
    }

    if (!frame->has_valid_bus_timestamp())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Kernel CAN receive timestamp unavailable on %s; published stamps "
          "use a local system-clock fallback and are marked invalid",
          can_channel_.c_str());
    }

    const int frame_sensor_id = Get_SensorID_From_MsgID(frame->get_id());
    auto sensor_it = sensor_id_to_local_index_.find(frame_sensor_id);
    if (sensor_it == sensor_id_to_local_index_.end())
    {
      return;
    }
    const size_t local_index = sensor_it->second;
    const canid_t base_message_id = Get_MsgID0_From_MsgID(frame->get_id());

    // When a filter configuration message is sent, the sensor replies with the messages
    // FilterState_Header (0x203) with the number of configured filters and one message FilterState_Cfg
    // (0x204) for the filter that has been changed.
    if (base_message_id == ID_FilterState_Cfg)
    {
      updateFilterConfig(frame, local_index);
      return;
    }

    if (base_message_id == ID_RadarState)
    {
      publishRadarState(frame, local_index);
      return;
    }

    // Dispatch from the received CAN message type itself. RadarState is useful
    // metadata, but it must not gate data that has already arrived on the bus.
    if (base_message_id == ID_Cluster_0_Status ||
        base_message_id == ID_Cluster_1_General)
    {
      handle_cluster_list(frame);
      return;
    }
  }

  void radar_conti_srr308::handle_object_list(std::shared_ptr<const polymath::socketcan::CanFrame> frame)
  {

    const int frame_sensor_id = Get_SensorID_From_MsgID(frame->get_id());
    auto sensor_it = sensor_id_to_local_index_.find(frame_sensor_id);
    if (sensor_it == sensor_id_to_local_index_.end())
    {
      return;
    }
    const size_t local_index = sensor_it->second;

    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_0_Status)
    {
      publish_object_map(local_index);
      object_list_list_[local_index].header.stamp = frame_bus_time(*frame);
      object_list_list_[local_index].object_count.data = GET_Obj_0_Status_Obj_NofObjects(frame->get_data());
      object_map_list_[local_index].clear();
    }

    // Object General Information
    // for each Obj_1_General message a new object has to be created in the map
    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_1_General)
    {

      radar_conti_srr308_msgs::msg::Object o;

      // object ID
      int id = GET_Obj_1_General_Obj_ID(frame->get_data());
      o.obj_id.data = GET_Obj_1_General_Obj_ID(frame->get_data());

      o.sensor_id.data = frame_sensor_id;

      // longitudinal distance
      o.object_general.obj_distlong.data =
          CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(frame->get_data()), 1.0);

      // lateral distance
      o.object_general.obj_distlat.data =
          CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(frame->get_data()), 1.0);

      // relative longitudinal velocity
      o.object_general.obj_vrellong.data =
          CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(frame->get_data()), 1.0);

      // relative lateral velocity
      o.object_general.obj_vrellat.data =
          CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(frame->get_data()), 1.0);

      o.object_general.obj_dynprop.data =
          CALC_Obj_1_General_Obj_DynProp(GET_Obj_1_General_Obj_DynProp(frame->get_data()), 1.0);

      o.object_general.obj_rcs.data =
          CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(frame->get_data()), 1.0);

      // insert object into map
      object_map_list_[local_index].insert(std::pair<int, radar_conti_srr308_msgs::msg::Object>(id, o));
    }

    // Object Quality Information
    // for each Obj_2_Quality message the existing object in the map has to be updated
    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_2_Quality)
    {

      // //RCLCPP_DEBUG(this->get_logger(), "Received Object_2_Quality msg (0x60c)");

      int id = GET_Obj_2_Quality_Obj_ID(frame->get_data());

      object_map_list_[local_index][id].object_quality.obj_distlong_rms.data =
          CALC_Obj_2_Quality_Obj_DistLong_rms(GET_Obj_2_Quality_Obj_DistLong_rms(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_quality.obj_distlat_rms.data =
          CALC_Obj_2_Quality_Obj_DistLat_rms(GET_Obj_2_Quality_Obj_DistLat_rms(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_quality.obj_vrellong_rms.data =
          CALC_Obj_2_Quality_Obj_VrelLong_rms(GET_Obj_2_Quality_Obj_VrelLong_rms(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_quality.obj_vrellat_rms.data =
          CALC_Obj_2_Quality_Obj_VrelLat_rms(GET_Obj_2_Quality_Obj_VrelLat_rms(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_quality.obj_probofexist.data =
          CALC_Obj_2_Quality_Obj_ProbOfExist(GET_Obj_2_Quality_Obj_ProbOfExist(frame->get_data()), 1.0);
    }

    // Object Extended Information
    // for each Obj_3_ExtInfo message the existing object in the map has to be updated
    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_3_Extended)
    {
      int id = GET_Obj_3_Extended_Obj_ID(frame->get_data());

      object_map_list_[local_index][id].object_extended.obj_arellong.data =
          CALC_Obj_3_Extended_Obj_ArelLong(GET_Obj_3_Extended_Obj_ArelLong(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_extended.obj_arellat.data =
          CALC_Obj_3_Extended_Obj_ArelLat(GET_Obj_3_Extended_Obj_ArelLat(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_extended.obj_class.data =
          CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_extended.obj_orientationangle.data =
          CALC_Obj_3_Extended_Obj_OrientationAngle(GET_Obj_3_Extended_Obj_OrientationAngle(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_extended.obj_length.data =
          CALC_Obj_3_Extended_Obj_Length(GET_Obj_3_Extended_Obj_Length(frame->get_data()), 1.0);

      object_map_list_[local_index][id].object_extended.obj_width.data =
          CALC_Obj_3_Extended_Obj_Width(GET_Obj_3_Extended_Obj_Width(frame->get_data()), 1.0);

      object_count = object_count + 1;
    };
  }

  // SRR308 Change: Add cluster handling
  void radar_conti_srr308::handle_cluster_list(std::shared_ptr<const polymath::socketcan::CanFrame> frame)
  {
    const int frame_sensor_id = Get_SensorID_From_MsgID(frame->get_id());
    auto sensor_it = sensor_id_to_local_index_.find(frame_sensor_id);
    if (sensor_it == sensor_id_to_local_index_.end())
    {
      return;
    }
    const size_t local_index = sensor_it->second;
    const canid_t base_message_id = Get_MsgID0_From_MsgID(frame->get_id());
    const rclcpp::Time bus_stamp = frame_bus_time(*frame);

    std::lock_guard<std::mutex> lock(cluster_scan_mutex_);
    auto &scan = cluster_scans_[local_index];

    if (base_message_id == ID_Cluster_0_Status)
    {
      if (frame->get_len() < DLC_Cluster_0_Status)
      {
        RCLCPP_WARN(
            this->get_logger(), "Ignoring short Cluster Status from sensor %d: DLC=%u",
            frame_sensor_id, static_cast<unsigned int>(frame->get_len()));
        return;
      }

      const uint16_t measurement_counter =
          GET_Cluster_0_Status_Cluster_MeasCounter(frame->get_data());

      // Every received Status is an explicit scan boundary. Do not infer that
      // equal counters are retransmissions: preserving the CAN stream is more
      // important than suppressing an unusual empty/partial scan.
      if (scan.active)
      {
        publish_cluster_scan(local_index);
      }

      scan = ClusterScanState{};
      scan.active = true;
      scan.status_valid = true;
      scan.measurement_counter = measurement_counter;
      scan.interface_version = static_cast<uint8_t>(
          GET_Cluster_0_Status_Cluster_InterfaceVersion(frame->get_data()));
      scan.expected_near_count = static_cast<uint16_t>(
          GET_Cluster_0_Status_Cluster_NofClustersNear(frame->get_data()));
      scan.expected_far_count = static_cast<uint16_t>(
          GET_Cluster_0_Status_Cluster_NofClustersFar(frame->get_data()));
      scan.scan_stamp = bus_stamp;
      scan.scan_stamp_valid = frame->has_valid_bus_timestamp();
      scan.all_rx_timestamps_valid = frame->has_valid_bus_timestamp();
      scan.first_rx_stamp = bus_stamp;
      scan.last_rx_stamp = bus_stamp;
      scan.last_receive_time = frame->get_receive_time();
      return;
    }

    // Cluster General Information, coordinates, speed and RCS
    if (base_message_id == ID_Cluster_1_General)
    {
      if (frame->get_len() < DLC_Cluster_1_General)
      {
        RCLCPP_WARN(
            this->get_logger(), "Ignoring short Cluster General from sensor %d: DLC=%u",
            frame_sensor_id, static_cast<unsigned int>(frame->get_len()));
        return;
      }

      // Preserve a General frame even if the corresponding Status was lost.
      // Such a scan is published with status_valid=false.
      if (!scan.active)
      {
        scan = ClusterScanState{};
        scan.active = true;
        scan.scan_stamp = bus_stamp;
        scan.scan_stamp_valid = frame->has_valid_bus_timestamp();
        scan.all_rx_timestamps_valid = frame->has_valid_bus_timestamp();
        scan.first_rx_stamp = bus_stamp;
      }

      // A lost Status under sustained traffic must not permit unbounded memory
      // growth. Publish the raw frames collected so far, then start another
      // explicitly status-less chunk. The normal SRR308 maximum is 510
      // General frames (255 near + 255 far), hence the default limit of 512.
      if (scan.clusters.size() >= max_cluster_frames_per_scan_)
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Cluster scan on %s sensor %d reached safety limit %zu; "
            "publishing a status-less chunk",
            can_channel_.c_str(), frame_sensor_id, max_cluster_frames_per_scan_);
        publish_cluster_scan(local_index);
        scan.active = true;
        scan.scan_stamp = bus_stamp;
        scan.scan_stamp_valid = frame->has_valid_bus_timestamp();
        scan.all_rx_timestamps_valid = frame->has_valid_bus_timestamp();
        scan.first_rx_stamp = bus_stamp;
      }

      radar_conti_srr308_msgs::msg::Cluster c;
      c.cluster_id.data = GET_Cluster_1_General_Cluster_ID(frame->get_data());
      c.sensor_id.data = frame_sensor_id;
      c.rx_stamp = bus_stamp;
      c.rx_stamp_valid = frame->has_valid_bus_timestamp();

      // longitudinal distance
      c.cluster_general.cluster_distlong.data =
          CALC_Cluster_1_General_Cluster_DistLong(GET_Cluster_1_General_Cluster_DistLong(frame->get_data()), 1.0);

      // lateral distance
      c.cluster_general.cluster_distlat.data =
          CALC_Cluster_1_General_Cluster_DistLat(GET_Cluster_1_General_Cluster_DistLat(frame->get_data()), 1.0);

      // relative longitudinal velocity
      c.cluster_general.cluster_vrellong.data =
          CALC_Cluster_1_General_Cluster_VrelLong(GET_Cluster_1_General_Cluster_VrelLong(frame->get_data()), 1.0);

      // relative lateral velocity
      c.cluster_general.cluster_vrellat.data =
          CALC_Cluster_1_General_Cluster_VrelLat(GET_Cluster_1_General_Cluster_VrelLat(frame->get_data()), 1.0);

      c.cluster_general.cluster_dynprop.data =
          CALC_Cluster_1_General_Cluster_DynProp(GET_Cluster_1_General_Cluster_DynProp(frame->get_data()), 1.0);

      c.cluster_general.cluster_rcs.data =
          CALC_Cluster_1_General_Cluster_RCS(GET_Cluster_1_General_Cluster_RCS(frame->get_data()), 1.0);

      // Keep CAN receive order and repeated IDs. Cluster ID is only a local
      // number within one radar scan and must not be used as a storage key.
      scan.clusters.push_back(c);
      scan.unique_cluster_ids.insert(c.cluster_id.data);
      scan.last_rx_stamp = bus_stamp;
      scan.all_rx_timestamps_valid =
          scan.all_rx_timestamps_valid && frame->has_valid_bus_timestamp();
      scan.last_receive_time = frame->get_receive_time();
    }
  }

  void radar_conti_srr308::publish_object_map(int sensor_id)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    radar_msgs::msg::RadarTracks radar_tracks;
    nav2_dynamic_msgs::msg::ObstacleArray obstacle_array;

    radar_tracks.header = object_list_list_[sensor_id].header;
    radar_tracks.header.frame_id = radar_link_names_[sensor_id];

    obstacle_array.header = radar_tracks.header;

    marker_array.markers.clear();

    // delete old marker
    visualization_msgs::msg::Marker ma;
    ma.action = 3;
    marker_array.markers.push_back(ma);
    marker_array_publishers_[sensor_id]->publish(marker_array);
    marker_array.markers.clear();

    tf2::Quaternion myQuaternion;

    std::map<int, radar_conti_srr308_msgs::msg::Object>::iterator itr;

    nav_msgs::msg::Odometry corrected_odom;
    if (!odom_topic_name_.empty())
    {
      corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry_, tf_buffer_, radar_link_names_[sensor_id], robot_base_frame_, transform_timeout_, this->get_clock());
    }

    for (itr = object_map_list_[sensor_id].begin(); itr != object_map_list_[sensor_id].end(); ++itr)
    {

      visualization_msgs::msg::Marker mobject;

      mobject.header = object_list_list_[sensor_id].header;
      mobject.header.frame_id = radar_link_names_[sensor_id];
      mobject.ns = "track";
      mobject.id = itr->first;
      mobject.type = 1;   // Cube
      mobject.action = 0; // add/modify
      mobject.pose.position.x = itr->second.object_general.obj_distlong.data;
      mobject.pose.position.y = itr->second.object_general.obj_distlat.data;

      double yaw = itr->second.object_extended.obj_orientationangle.data * 3.1416 / 180.0;

      myQuaternion.setRPY(0, 0, yaw);
      mobject.pose.orientation.w = myQuaternion.getW();
      mobject.pose.orientation.x = myQuaternion.getX();
      mobject.pose.orientation.y = myQuaternion.getY();
      mobject.pose.orientation.z = myQuaternion.getZ();
      // SRR308 Change: To draw different color of cube for different radar
      // mobject.color.r = 0.0;
      // mobject.color.g = 1.0;
      // mobject.color.b = 0.0;
      // mobject.color.a = 1.0;
      if (sensor_id == 0) {
      mobject.color.r = 1.0; mobject.color.g = 0.0; mobject.color.b = 0.0; // left front radar
      } else if (sensor_id == 1) {
      mobject.color.r = 0.0; mobject.color.g = 0.0; mobject.color.b = 1.0; // right front radar
      }
      mobject.color.a = 1.0;
      // *******************************************************
      mobject.lifetime = rclcpp::Duration::from_seconds(0.2);
      mobject.frame_locked = false;

      radar_msgs::msg::RadarTrack radar_track;
      radar_track.uuid = UUID_table_[itr->first + max_radar_id * sensor_id];
      radar_track.position.x = (itr->second.object_extended.obj_length.data / 2) + itr->second.object_general.obj_distlong.data;
      radar_track.position.y = itr->second.object_general.obj_distlat.data;
      radar_track.position.z = 0.0;

      geometry_msgs::msg::Vector3 radar_track_vel_raw;
      radar_track_vel_raw.x = itr->second.object_general.obj_vrellong.data;
      radar_track_vel_raw.y = itr->second.object_general.obj_vrellat.data;
      radar_track_vel_raw.z = 0.0;
      radar_track.velocity = radar_transforms::correctObstacleVelocity(corrected_odom, radar_track_vel_raw, radar_track.position);

      radar_track.acceleration.x = 0.0;
      radar_track.acceleration.y = 0.0;
      radar_track.acceleration.z = 0.0;

      // radar_track.size.x = itr->second.object_extended.obj_length.data;
      // radar_track.size.y = itr->second.object_extended.obj_width.data;
      // radar_track.size.z = 1.0;

      // SRR308 Change: To draw a bigger cube for visualizaiton.
      radar_track.size.x = std::max(0.5, itr->second.object_extended.obj_length.data);
      radar_track.size.y = std::max(0.5, itr->second.object_extended.obj_width.data);
      radar_track.size.z = 1.0;
      // ###################################################

      nav2_dynamic_msgs::msg::Obstacle obstacle;
      obstacle.score = itr->second.object_quality.obj_probofexist.data;
      obstacle.uuid = radar_track.uuid;
      obstacle.position = radar_track.position;
      obstacle.velocity = radar_track.velocity;
      obstacle.size = radar_track.size;

      obstacle.position_covariance[0] = covariance[static_cast<int>(itr->second.object_quality.obj_distlong_rms.data)];
      obstacle.position_covariance[1] = 0.0;
      obstacle.position_covariance[2] = 0.0;
      obstacle.position_covariance[3] = 0.0;
      obstacle.position_covariance[4] = covariance[static_cast<int>(itr->second.object_quality.obj_distlat_rms.data)];
      obstacle.position_covariance[5] = 0.0;
      obstacle.position_covariance[6] = 0.0;
      obstacle.position_covariance[7] = 0.0;
      obstacle.position_covariance[8] = 0.0;

      obstacle.velocity_covariance[0] = covariance[static_cast<int>(itr->second.object_quality.obj_vrellong_rms.data)];
      obstacle.velocity_covariance[1] = 0.0;
      obstacle.velocity_covariance[2] = 0.0;
      obstacle.velocity_covariance[3] = 0.0;
      obstacle.velocity_covariance[4] = covariance[static_cast<int>(itr->second.object_quality.obj_vrellat_rms.data)];
      obstacle.velocity_covariance[5] = 0.0;
      obstacle.velocity_covariance[6] = 0.0;
      obstacle.velocity_covariance[7] = 0.0;
      obstacle.velocity_covariance[8] = 0.0;

      mobject.pose.position = radar_track.position;
      mobject.scale = radar_track.size;

      visualization_msgs::msg::Marker arrow_vrelong = mobject;
      arrow_vrelong.type = 0; // Arrow
      // SRR308 Change: Scale the velocity arrows for better visualization.
      // arrow_vrelong.scale.x = radar_track.velocity.x;
      arrow_vrelong.scale.x = std::max(0.1, std::abs(radar_track.velocity.x) * 0.2);
      // *******************************************************
      arrow_vrelong.scale.y = 0.1;
      arrow_vrelong.scale.z = 0.1;
      arrow_vrelong.ns = "vrelong_arrow";
      arrow_vrelong.color.r = 1.0;
      arrow_vrelong.color.g = 0.0;
      arrow_vrelong.color.b = 0.0;
      arrow_vrelong.color.a = 1.0;

      visualization_msgs::msg::Marker arrow_vrelat = mobject;
      auto vrelat_quat = myQuaternion;
      vrelat_quat.setRPY(0, 0, yaw + M_PI_2);
      arrow_vrelat.pose.orientation.w = vrelat_quat.getW();
      arrow_vrelat.pose.orientation.x = vrelat_quat.getX();
      arrow_vrelat.pose.orientation.y = vrelat_quat.getY();
      arrow_vrelat.pose.orientation.z = vrelat_quat.getZ();

      arrow_vrelat.type = 0; // Arrow
      // SRR308 Change: Scale the velocity arrows for better visualization.
      // arrow_vrelat.scale.x = radar_track.velocity.y;
      arrow_vrelat.scale.x = std::max(0.1, std::abs(radar_track.velocity.y) * 0.2);
      // *******************************************************
      arrow_vrelat.scale.y = 0.1;
      arrow_vrelat.scale.z = 0.1;
      arrow_vrelat.ns = "vrelat_arrow";
      arrow_vrelat.color.r = 0.0;
      arrow_vrelat.color.g = 1.0;
      arrow_vrelat.color.b = 0.0;
      arrow_vrelat.color.a = 1.0;

      marker_array.markers.push_back(mobject);
      marker_array.markers.push_back(arrow_vrelong);
      marker_array.markers.push_back(arrow_vrelat);

      radar_tracks.tracks.push_back(radar_track);
      obstacle_array.obstacles.push_back(obstacle);

      // SRR308 Change: fix object list topic publisher
      object_list_list_[sensor_id].objects.push_back(itr->second);

    }

    marker_array_publishers_[sensor_id]->publish(marker_array);
    radar_tracks_publishers_[sensor_id]->publish(radar_tracks);
    obstacle_array_publishers_[sensor_id]->publish(obstacle_array);

    // SRR308 Change: fix object list topic publisher
    object_list_publishers_[sensor_id]->publish(object_list_list_[sensor_id]);
    object_list_list_[sensor_id].objects.clear();    

  }

  void radar_conti_srr308::setFilterService(
      const std::shared_ptr<radar_conti_srr308_msgs::srv::SetFilter::Request> request,
      std::shared_ptr<radar_conti_srr308_msgs::srv::SetFilter::Response> response)
  {
    if (!enable_manual_can_tx_)
    {
      response->success = false;
      response->sent = false;
      response->confirmed = false;
      response->message = "enable_manual_can_tx is false; filter CAN write skipped";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::lock_guard<std::mutex> tx_lock(can_tx_mutex_);
    const auto result = setFilter(
        request->sensor_id, request->active, request->type, request->index,
        request->min_value, request->max_value);
    response->success = result.sent && result.confirmed;
    response->sent = result.sent;
    response->confirmed = result.confirmed;
    response->message = result.message;
  }

  void radar_conti_srr308::setFilterConfigurationService(
      const std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Request> request,
      std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Response> response)
  {
    if (!enable_manual_can_tx_)
    {
      response->success = false;
      response->message = "enable_manual_can_tx is false; filter configuration CAN writes skipped";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    const auto sensor_it = sensor_id_to_local_index_.find(request->sensor_id);
    if (sensor_it == sensor_id_to_local_index_.end())
    {
      response->success = false;
      response->message = fmt::format(
          "Sensor ID '{}' is not configured on {}", request->sensor_id, can_channel_);
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::lock_guard<std::mutex> tx_lock(can_tx_mutex_);
    response->success = applyFilterConfiguration(sensor_it->second, response->message);
    if (!response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  bool radar_conti_srr308::applyFilterConfiguration(
      const std::size_t local_index, std::string &message)
  {
    if (local_index >= desired_filter_configs_.size() ||
        local_index >= radar_filter_active_.size() ||
        local_index >= radar_sensor_ids_.size())
    {
      message = fmt::format("Invalid local radar index {}", local_index);
      return false;
    }

    const int sensor_id = radar_sensor_ids_[local_index];

    // Preflight the complete YAML request before the first CAN write. A
    // deterministic type/range error must not leave the radar half configured.
    for (const uint8_t filter_index : kSupportedClusterFilterIndices)
    {
      const auto values = get_filter_values(
          desired_filter_configs_[local_index], filterTypes[filter_index]);
      const bool active = radar_filter_active_[local_index][filter_index];
      FilterEncoding encoding;
      std::string validation_message;
      if (!validate_filter_request(
              active, FilterCfg_FilterCfg_Type_Cluster, filter_index,
              values.min_value, values.max_value, encoding,
              validation_message))
      {
        message = fmt::format(
            "Filter configuration preflight failed for sensor {} index {} "
            "before any CAN frame was sent: {}",
            sensor_id, filter_index, validation_message);
        return false;
      }
    }

    std::size_t applied_count = 0;
    std::string applied_indices;
    for (std::size_t supported_index = 0;
         supported_index < kSupportedClusterFilterIndices.size();
         ++supported_index)
    {
      const uint8_t filter_index =
          kSupportedClusterFilterIndices[supported_index];
      const auto values = get_filter_values(
          desired_filter_configs_[local_index], filterTypes[filter_index]);
      const bool active = radar_filter_active_[local_index][filter_index];
      const auto result = setFilter(
          sensor_id, active, FilterCfg_FilterCfg_Type_Cluster, filter_index,
          values.min_value, values.max_value);
      if (!result.sent || !result.confirmed)
      {
        if (result.sent)
        {
          message = fmt::format(
              "Filter configuration stopped at sensor {} index {}: {}. "
              "The failed index was sent but not confirmed and its radar "
              "state is unknown. Previously confirmed indices [{}] remain "
              "applied; this batch service cannot roll back radar state",
              sensor_id, filter_index, result.message, applied_indices);
        }
        else
        {
          message = fmt::format(
              "Filter configuration stopped before sending sensor {} index "
              "{}: {}. Previously confirmed indices [{}] remain applied; "
              "this batch service cannot roll back radar state",
              sensor_id, filter_index, result.message, applied_indices);
        }
        return false;
      }
      if (!applied_indices.empty())
      {
        applied_indices += ",";
      }
      applied_indices += std::to_string(filter_index);
      ++applied_count;

      // Keep an explicit gap between configuration frames even when the
      // radar acknowledges immediately. This service is deliberately manual
      // and must not recreate the previous CAN transmit burst.
      if (supported_index + 1U < kSupportedClusterFilterIndices.size())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    publishFilterConfigMetadata();

    message = fmt::format(
        "Applied and confirmed {} supported Cluster filters for sensor {}; "
        "non-Cluster/unsupported indices 3,4,6,7,8,9,10,11,12,13,14 were skipped",
        applied_count, sensor_id);
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    return true;
  }

  radar_conti_srr308::FilterCommandResult radar_conti_srr308::setFilter(
      const int sensor_id, const bool active, const uint8_t type,
      const uint8_t index, const double min_value, const double max_value)
  {
    FilterCommandResult result;
    if (!enable_can_tx_ && !enable_manual_can_tx_)
    {
      result.message = "CAN transmit is disabled; filter configuration was not sent";
      return result;
    }

    const auto sensor_it = sensor_id_to_local_index_.find(sensor_id);
    if (sensor_it == sensor_id_to_local_index_.end())
    {
      result.message = fmt::format(
          "Sensor ID '{}' is not configured on {}", sensor_id, can_channel_);
      return result;
    }
    const std::size_t local_index = sensor_it->second;

    FilterEncoding encoding;
    if (!validate_filter_request(
            active, type, index, min_value, max_value, encoding,
            result.message))
    {
      return result;
    }
    if (!socketcan_adapter_ ||
        socketcan_adapter_->get_socket_state() != polymath::socketcan::SocketState::OPEN)
    {
      result.message = fmt::format(
          "CAN socket '{}' is not active; filter configuration was not sent",
          can_channel_);
      return result;
    }

    const auto filter_type = encoding.filter_type;
    const long raw_min = encoding.raw_min;
    const long raw_max = encoding.raw_max;

    polymath::socketcan::CanFrame frame;
    frame.set_len(DLC_FilterCfg);

    uint32_t msg_id = ID_FilterCfg;
    Set_SensorID_In_MsgID(msg_id, sensor_id);
    frame.set_can_id(msg_id);

    std::array<unsigned char, CAN_MAX_DLC> data{0};
    SET_FilterCfg_FilterCfg_Active(
        data, active ? FilterCfg_FilterCfg_Active_active : FilterCfg_FilterCfg_Active_inactive);
    SET_FilterCfg_FilterCfg_Valid(data, 1);
    SET_FilterCfg_FilterCfg_Type(data, type);
    SET_FilterCfg_FilterCfg_Index(data, index);

    switch (filter_type)
    {
    case FilterType::NOFOBJ:
      SET_FilterCfg_FilterCfg_Max_NofObj(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_NofObj(data, raw_min);
      break;
    case FilterType::DISTANCE:
      SET_FilterCfg_FilterCfg_Max_Distance(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_Distance(data, raw_min);
      break;
    case FilterType::AZIMUTH:
      SET_FilterCfg_FilterCfg_Max_Azimuth(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_Azimuth(data, raw_min);
      break;
    case FilterType::VRELONCOME:
      SET_FilterCfg_FilterCfg_Max_VrelOncome(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_VrelOncome(data, raw_min);
      break;
    case FilterType::VRELDEPART:
      SET_FilterCfg_FilterCfg_Max_VrelDepart(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_VrelDepart(data, raw_min);
      break;
    case FilterType::RCS:
      SET_FilterCfg_FilterCfg_Max_RCS(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_RCS(data, raw_min);
      break;
    case FilterType::LIFETIME:
      SET_FilterCfg_FilterCfg_Max_Lifetime(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_Lifetime(data, raw_min);
      break;
    case FilterType::SIZE:
      SET_FilterCfg_FilterCfg_Max_Size(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_Size(data, raw_min);
      break;
    case FilterType::PROBEXISTS:
      SET_FilterCfg_FilterCfg_Max_ProbExists(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_ProbExists(data, raw_min);
      break;
    case FilterType::Y:
      SET_FilterCfg_FilterCfg_Max_Y(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_Y(data, raw_min);
      break;
    case FilterType::X:
      SET_FilterCfg_FilterCfg_Max_X(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_X(data, raw_min);
      break;
    case FilterType::VYRIGHTLEFT:
      SET_FilterCfg_FilterCfg_Max_VYRightLeft(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_VYRightLeft(data, raw_min);
      break;
    case FilterType::VXONCOME:
      SET_FilterCfg_FilterCfg_Max_VXOncome(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_VXOncome(data, raw_min);
      break;
    case FilterType::VYLEFTRIGHT:
      SET_FilterCfg_FilterCfg_Max_VYLeftRight(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_VYLeftRight(data, raw_min);
      break;
    case FilterType::VXDEPART:
      SET_FilterCfg_FilterCfg_Max_VXDepart(data, raw_max);
      SET_FilterCfg_FilterCfg_Min_VXDepart(data, raw_min);
      break;
    default:
      result.message = fmt::format("Unknown filter index {}", index);
      return result;
    }

    frame.set_data(data);

    {
      std::lock_guard<std::mutex> confirmation_lock(filter_confirmation_mutex_);
      pending_filter_confirmation_ = PendingFilterConfirmation{};
      pending_filter_confirmation_.waiting = true;
      pending_filter_confirmation_.local_index = local_index;
      pending_filter_confirmation_.type = type;
      pending_filter_confirmation_.index = index;
      pending_filter_confirmation_.active = active;
      pending_filter_confirmation_.compare_limits = active;
      pending_filter_confirmation_.min_value = encoding.expected_min;
      pending_filter_confirmation_.max_value = encoding.expected_max;
      pending_filter_confirmation_.bus_not_before =
          std::chrono::system_clock::now();
      pending_filter_confirmation_.receive_not_before =
          std::chrono::steady_clock::now();
    }

    auto err = socketcan_adapter_->send(frame);
    if (err.has_value())
    {
      {
        std::lock_guard<std::mutex> confirmation_lock(filter_confirmation_mutex_);
        pending_filter_confirmation_ = PendingFilterConfirmation{};
      }
      result.message = fmt::format(
          "Error sending FilterCfg for sensor {} index {}: {}",
          sensor_id, index, err.value());
      return result;
    }
    result.sent = true;

    std::unique_lock<std::mutex> confirmation_lock(filter_confirmation_mutex_);
    const bool received = filter_confirmation_cv_.wait_for(
        confirmation_lock, filter_confirmation_timeout_,
        [this]()
        {
          return pending_filter_confirmation_.received;
        });
    if (!received)
    {
      const bool saw_mismatch = pending_filter_confirmation_.saw_mismatch;
      pending_filter_confirmation_ = PendingFilterConfirmation{};
      if (saw_mismatch)
      {
        result.message = fmt::format(
            "FilterCfg for sensor {} index {} was sent with DLC {}; fresh "
            "FilterState_Cfg replies arrived, but none matched within {} ms",
            sensor_id, index, DLC_FilterCfg,
            filter_confirmation_timeout_.count());
      }
      else
      {
        result.message = fmt::format(
            "FilterCfg for sensor {} index {} was sent with DLC {}, but no "
            "matching FilterState_Cfg reply arrived within {} ms",
            sensor_id, index, DLC_FilterCfg,
            filter_confirmation_timeout_.count());
      }
      return result;
    }

    const bool cancelled = pending_filter_confirmation_.cancelled;
    result.confirmed = pending_filter_confirmation_.matched;
    pending_filter_confirmation_ = PendingFilterConfirmation{};
    if (cancelled)
    {
      result.message = fmt::format(
          "FilterCfg for sensor {} index {} was sent, but confirmation wait "
          "was interrupted by lifecycle deactivation/shutdown",
          sensor_id, index);
      return result;
    }
    if (!result.confirmed)
    {
      result.message = fmt::format(
          "Sensor {} returned FilterState_Cfg for index {}, but active/limits "
          "did not match the request",
          sensor_id, index);
      return result;
    }

    result.message = fmt::format(
        "Filter sensor {} index {} active={} sent and confirmed",
        sensor_id, index, active);
    return result;
  }

  void radar_conti_srr308::publish_cluster_scan(std::size_t local_index)
  {
    auto &scan = cluster_scans_[local_index];
    if (!scan.active)
    {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    radar_conti_srr308_msgs::msg::ClusterList out_cluster_list;
    out_cluster_list.header.stamp = scan.scan_stamp;
    out_cluster_list.header.frame_id = radar_link_names_[local_index];
    out_cluster_list.can_channel = can_channel_;
    out_cluster_list.sensor_id = static_cast<uint8_t>(radar_sensor_ids_[local_index]);
    out_cluster_list.measurement_counter = scan.measurement_counter;
    out_cluster_list.interface_version = scan.interface_version;
    out_cluster_list.expected_near_count = scan.expected_near_count;
    out_cluster_list.expected_far_count = scan.expected_far_count;
    out_cluster_list.received_count = static_cast<uint32_t>(scan.clusters.size());
    out_cluster_list.unique_cluster_count =
        static_cast<uint32_t>(scan.unique_cluster_ids.size());
    out_cluster_list.duplicate_count =
        out_cluster_list.received_count - out_cluster_list.unique_cluster_count;
    out_cluster_list.status_valid = scan.status_valid;
    const std::size_t expected_count =
        static_cast<std::size_t>(scan.expected_near_count) +
        static_cast<std::size_t>(scan.expected_far_count);
    out_cluster_list.complete =
        scan.status_valid && scan.clusters.size() == expected_count;
    out_cluster_list.header_stamp_valid = scan.scan_stamp_valid;
    out_cluster_list.all_rx_stamps_valid = scan.all_rx_timestamps_valid;
    out_cluster_list.first_rx_stamp = scan.first_rx_stamp;
    out_cluster_list.last_rx_stamp = scan.last_rx_stamp;
    out_cluster_list.clusters = scan.clusters;
    out_cluster_list.cluster_count.data =
        static_cast<int32_t>(out_cluster_list.clusters.size());

    if (!out_cluster_list.complete)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Publishing partial cluster scan on %s sensor %d: status_valid=%s "
          "counter=%u expected=%zu received=%zu",
          can_channel_.c_str(), radar_sensor_ids_[local_index],
          scan.status_valid ? "true" : "false", scan.measurement_counter,
          expected_count, scan.clusters.size());
    }

    visualization_msgs::msg::Marker cluster_marker;
    cluster_marker.header = out_cluster_list.header;
    cluster_marker.ns = radar_link_names_[local_index] + "/clusters";
    cluster_marker.id = 0;
    cluster_marker.pose.orientation.w = 1.0;
    cluster_marker.scale.x = cluster_marker_scale_;
    cluster_marker.scale.y = cluster_marker_scale_;
    cluster_marker.scale.z = cluster_marker_scale_;
    cluster_marker.lifetime = rclcpp::Duration::from_seconds(cluster_marker_lifetime_sec_);
    cluster_marker.frame_locked = true;

    switch (radar_sensor_ids_[local_index])
    {
      case 0:
        cluster_marker.color.r = 1.0;
        break;
      case 1:
        cluster_marker.color.b = 1.0;
        break;
      case 2:
        cluster_marker.color.g = 1.0;
        break;
      case 3:
        cluster_marker.color.r = 1.0;
        cluster_marker.color.g = 1.0;
        break;
      default:
        cluster_marker.color.r = 1.0;
        cluster_marker.color.b = 1.0;
        break;
    }
    cluster_marker.color.a = 1.0;

    for (const auto &cluster : scan.clusters)
    {
      geometry_msgs::msg::Point point;
      point.x = cluster.cluster_general.cluster_distlong.data;
      point.y = cluster.cluster_general.cluster_distlat.data;
      point.z = 0.0;
      cluster_marker.points.push_back(point);
    }

    if (!scan.clusters.empty())
    {
      cluster_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      cluster_marker.action = visualization_msgs::msg::Marker::ADD;
      marker_array.markers.push_back(cluster_marker);
    }
    else
    {
      cluster_marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(cluster_marker);
    }

    marker_array_publishers_[local_index]->publish(marker_array);
    cluster_list_publishers_[local_index]->publish(out_cluster_list);
    scan = ClusterScanState{};
  }

  void radar_conti_srr308::check_cluster_scan_timeouts()
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(cluster_scan_mutex_);
    for (std::size_t local_index = 0; local_index < cluster_scans_.size(); ++local_index)
    {
      const auto &scan = cluster_scans_[local_index];
      if (scan.active &&
          scan.last_receive_time != std::chrono::steady_clock::time_point{} &&
          now - scan.last_receive_time >= cluster_scan_timeout_)
      {
        publish_cluster_scan(local_index);
      }
    }
  }

  void radar_conti_srr308::flush_pending_cluster_scans()
  {
    std::lock_guard<std::mutex> lock(cluster_scan_mutex_);
    for (std::size_t local_index = 0; local_index < cluster_scans_.size(); ++local_index)
    {
      publish_cluster_scan(local_index);
    }
  }

  void radar_conti_srr308::setRadarConfigurationService(
      const std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Request> request,
      std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Response> response)
  {
    auto req = *request;
    if (!enable_manual_can_tx_)
    {
      response->success = false;
      response->message = "enable_manual_can_tx is false; radar configuration CAN write skipped";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    std::lock_guard<std::mutex> tx_lock(can_tx_mutex_);
    if (!setRadarConfiguration(req.sensor_id, response))
    {
      response->success = false;
      return;
    }
    response->success = true;
    response->message = fmt::format(
        "RadarConfiguration frame sent for sensor {}", req.sensor_id);
  }

  bool radar_conti_srr308::setRadarConfiguration(const int &sensor_id, std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Response> &response)
  {
    if (!enable_can_tx_ && !enable_manual_can_tx_)
    {
      response->message = "CAN transmit is disabled; radar configuration CAN write skipped";
      RCLCPP_DEBUG(this->get_logger(), "%s", response->message.c_str());
      return true;
    }

    const auto sensor_it = sensor_id_to_local_index_.find(sensor_id);
    if (sensor_it == sensor_id_to_local_index_.end())
    {
      auto msg = fmt::format("Sensor ID '{}' not found in radar configurations.", sensor_id);
      response->message = msg;
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      return false;
    }
    if (!socketcan_adapter_ ||
        socketcan_adapter_->get_socket_state() != polymath::socketcan::SocketState::OPEN)
    {
      response->message =
          fmt::format("CAN socket '{}' is not active; configuration was not sent", can_channel_);
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Setting radar configuration for sensor_id: %i", sensor_id);
    polymath::socketcan::CanFrame frame;
    frame.set_len(DLC_RadarConfiguration);

    uint32_t msg_id = ID_RadarConfiguration;
    Set_SensorID_In_MsgID(msg_id, sensor_id);
    frame.set_can_id(msg_id);

    std::array<unsigned char, DLC_RadarConfiguration> data{0};

    const auto &current_radar_config = radar_configuration_configs_[sensor_it->second];

    // RADAR POWER CFG
    if (current_radar_config.radarcfg_radarpower.data > MAX_RadarConfiguration_RadarCfg_RadarPower)
    {
      std::string msg = fmt::format("Radar Power '{}' outside of range", current_radar_config.radarcfg_radarpower.data);
      response->message = msg;
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      return false;
    }

    // Radar Power
    SET_RadarConfiguration_RadarCfg_RadarPower_valid(data, current_radar_config.radarcfg_radarpower_valid.data);
    SET_RadarConfiguration_RadarCfg_RadarPower(data, current_radar_config.radarcfg_radarpower.data);

    // NVM Storage
    SET_RadarConfiguration_RadarCfg_StoreInNVM(data, current_radar_config.radarcfg_storeinnvm.data);
    SET_RadarConfiguration_RadarCfg_StoreInNVM_valid(data, current_radar_config.radarcfg_storeinnvm_valid.data);

    // Send Ext Info
    SET_RadarConfiguration_RadarCfg_SendExtInfo(data, current_radar_config.radarcfg_sendextinfo.data);
    SET_RadarConfiguration_RadarCfg_SendExtInfo_valid(data, current_radar_config.radarcfg_sendextinfo_valid.data);

    // Ctrl Relay
    SET_RadarConfiguration_RadarCfg_CtrlRelay(data, current_radar_config.radarcfg_ctrlrelay.data);
    SET_RadarConfiguration_RadarCfg_CtrlRelay_valid(data, current_radar_config.radarcfg_ctrlrelay_valid.data);

    // Send Quality
    SET_RadarConfiguration_RadarCfg_SendQuality(data, current_radar_config.radarcfg_sendquality.data);
    SET_RadarConfiguration_RadarCfg_SendQuality_valid(data, current_radar_config.radarcfg_sendquality_valid.data);

    // Max Distance - Resolution of 2m
    SET_RadarConfiguration_RadarCfg_MaxDistance(data, current_radar_config.radarcfg_maxdistance.data / 2);
    SET_RadarConfiguration_RadarCfg_MaxDistance_valid(data, current_radar_config.radarcfg_maxdistance_valid.data);

    // Sensor ID
    SET_RadarConfiguration_RadarCfg_SensorID(data, current_radar_config.radarcfg_sensorid.data);
    SET_RadarConfiguration_RadarCfg_SensorID_valid(data, current_radar_config.radarcfg_sensorid_valid.data);

    // RCS Threshold
    SET_RadarConfiguration_RadarCfg_RCS_Threshold(data, current_radar_config.radarcfg_rcs_threshold.data);
    SET_RadarConfiguration_RadarCfg_RCS_Threshold_Valid(data, current_radar_config.radarcfg_rcs_threshold_valid.data);

    // Output Type
    SET_RadarConfiguration_RadarCfg_OutputType(data, current_radar_config.radarcfg_outputtype.data);
    SET_RadarConfiguration_RadarCfg_OutputType_valid(data, current_radar_config.radarcfg_outputtype_valid.data);

    // Sort Index
    SET_RadarConfiguration_RadarCfg_SortIndex(data, current_radar_config.radarcfg_sortindex.data);
    SET_RadarConfiguration_RadarCfg_SortIndex_valid(data, current_radar_config.radarcfg_sortindex_valid.data);

    frame.set_data(data);
    auto err = socketcan_adapter_->send(frame);
    if (err.has_value())
    {
      auto msg = fmt::format("Error sending frame: {}", err.value());
      response->message = msg;
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());

      return false;
    }

    return true;
  }

  void radar_conti_srr308::updateFilterConfig(std::shared_ptr<const polymath::socketcan::CanFrame> frame, const int &sensor_id)
  {
    if (sensor_id < 0 ||
        static_cast<std::size_t>(sensor_id) >= radar_filter_configs_.size())
    {
      return;
    }
    if (frame->get_len() < DLC_FilterState_Cfg)
    {
      RCLCPP_WARN(
          this->get_logger(), "Ignoring short FilterState from sensor %d: DLC=%u",
          radar_sensor_ids_[sensor_id], static_cast<unsigned int>(frame->get_len()));
      return;
    }

    const uint8_t type = static_cast<uint8_t>(
        GET_FilterState_Cfg_FilterState_Type(frame->get_data()));
    const uint8_t index = static_cast<uint8_t>(
        GET_FilterState_Cfg_FilterState_Index(frame->get_data()));
    if (type != FilterCfg_FilterCfg_Type_Cluster ||
        index >= kConfiguredFilterCount ||
        !is_supported_cluster_filter(index))
    {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Ignoring unsupported FilterState_Cfg sensor=%d type=%u index=%u",
          radar_sensor_ids_[sensor_id], static_cast<unsigned int>(type),
          static_cast<unsigned int>(index));
      return;
    }

    FilterValues observed;
    {
      std::lock_guard<std::mutex> lock(filter_config_mutex_);
      radar_filter_configs_[sensor_id].header.stamp = frame_bus_time(*frame);
      radar_filter_configs_[sensor_id].header.frame_id = radar_link_names_[sensor_id];

      switch (filterTypes[index])
      {
    case FilterType::NOFOBJ:
      radar_filter_configs_[sensor_id].nofobj.min = CALC_FilterState_Cfg_FilterState_Min_NofObj(
          GET_FilterState_Cfg_FilterState_Min_NofObj(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].nofobj.max = CALC_FilterState_Cfg_FilterState_Max_NofObj(
          GET_FilterState_Cfg_FilterState_Max_NofObj(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].nofobj.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::DISTANCE:
      radar_filter_configs_[sensor_id].distance.min = CALC_FilterState_Cfg_FilterState_Min_Distance(
          GET_FilterState_Cfg_FilterState_Min_Distance(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].distance.max = CALC_FilterState_Cfg_FilterState_Max_Distance(
          GET_FilterState_Cfg_FilterState_Max_Distance(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].distance.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::AZIMUTH:
      radar_filter_configs_[sensor_id].azimuth.min = CALC_FilterState_Cfg_FilterState_Min_Azimuth(
          GET_FilterState_Cfg_FilterState_Min_Azimuth(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].azimuth.max = CALC_FilterState_Cfg_FilterState_Max_Azimuth(
          GET_FilterState_Cfg_FilterState_Max_Azimuth(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].azimuth.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VRELONCOME:
      radar_filter_configs_[sensor_id].vreloncome.min = CALC_FilterState_Cfg_FilterState_Min_VrelOncome(
          GET_FilterState_Cfg_FilterState_Min_VrelOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreloncome.max = CALC_FilterState_Cfg_FilterState_Max_VrelOncome(
          GET_FilterState_Cfg_FilterState_Max_VrelOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreloncome.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VRELDEPART:
      radar_filter_configs_[sensor_id].vreldepart.min = CALC_FilterState_Cfg_FilterState_Min_VrelDepart(
          GET_FilterState_Cfg_FilterState_Min_VrelDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreldepart.max = CALC_FilterState_Cfg_FilterState_Max_VrelDepart(
          GET_FilterState_Cfg_FilterState_Max_VrelDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreldepart.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::RCS:
      radar_filter_configs_[sensor_id].rcs.min = CALC_FilterState_Cfg_FilterState_Min_RCS(
          GET_FilterState_Cfg_FilterState_Min_RCS(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].rcs.max = CALC_FilterState_Cfg_FilterState_Max_RCS(
          GET_FilterState_Cfg_FilterState_Max_RCS(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].rcs.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::LIFETIME:
      radar_filter_configs_[sensor_id].lifetime.min = CALC_FilterState_Cfg_FilterState_Min_Lifetime(
          GET_FilterState_Cfg_FilterState_Min_Lifetime(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].lifetime.max = CALC_FilterState_Cfg_FilterState_Max_Lifetime(
          GET_FilterState_Cfg_FilterState_Max_Lifetime(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].lifetime.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::SIZE:
      radar_filter_configs_[sensor_id].size.min = CALC_FilterState_Cfg_FilterState_Min_Size(
          GET_FilterState_Cfg_FilterState_Min_Size(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].size.max = CALC_FilterState_Cfg_FilterState_Max_Size(
          GET_FilterState_Cfg_FilterState_Max_Size(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].size.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::PROBEXISTS:
      radar_filter_configs_[sensor_id].probexists.min = CALC_FilterState_Cfg_FilterState_Min_ProbExists(
          GET_FilterState_Cfg_FilterState_Min_ProbExists(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].probexists.max = CALC_FilterState_Cfg_FilterState_Max_ProbExists(
          GET_FilterState_Cfg_FilterState_Max_ProbExists(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].probexists.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::Y:
      radar_filter_configs_[sensor_id].y.min = CALC_FilterState_Cfg_FilterState_Min_Y(
          GET_FilterState_Cfg_FilterState_Min_Y(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].y.max = CALC_FilterState_Cfg_FilterState_Max_Y(
          GET_FilterState_Cfg_FilterState_Max_Y(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].y.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::X:
      radar_filter_configs_[sensor_id].x.min = CALC_FilterState_Cfg_FilterState_Min_X(
          GET_FilterState_Cfg_FilterState_Min_X(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].x.max = CALC_FilterState_Cfg_FilterState_Max_X(
          GET_FilterState_Cfg_FilterState_Max_X(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].x.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VYRIGHTLEFT:
      radar_filter_configs_[sensor_id].vyrightleft.min = CALC_FilterState_Cfg_FilterState_Min_VYRightLeft(
          GET_FilterState_Cfg_FilterState_Min_VYRightLeft(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyrightleft.max = CALC_FilterState_Cfg_FilterState_Max_VYRightLeft(
          GET_FilterState_Cfg_FilterState_Max_VYRightLeft(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyrightleft.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VXONCOME:
      radar_filter_configs_[sensor_id].vxoncome.min = CALC_FilterState_Cfg_FilterState_Min_VXOncome(
          GET_FilterState_Cfg_FilterState_Min_VXOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxoncome.max = CALC_FilterState_Cfg_FilterState_Max_VXOncome(
          GET_FilterState_Cfg_FilterState_Max_VXOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxoncome.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VYLEFTRIGHT:
      radar_filter_configs_[sensor_id].vyleftright.min = CALC_FilterState_Cfg_FilterState_Min_VYLeftRight(
          GET_FilterState_Cfg_FilterState_Min_VYLeftRight(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyleftright.max = CALC_FilterState_Cfg_FilterState_Max_VYLeftRight(
          GET_FilterState_Cfg_FilterState_Max_VYLeftRight(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyleftright.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VXDEPART:
      radar_filter_configs_[sensor_id].vxdepart.min = CALC_FilterState_Cfg_FilterState_Min_VXDepart(
          GET_FilterState_Cfg_FilterState_Min_VXDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxdepart.max = CALC_FilterState_Cfg_FilterState_Max_VXDepart(
          GET_FilterState_Cfg_FilterState_Max_VXDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxdepart.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
      default:
        break;
      }
      radar_filter_configs_[sensor_id].valid_mask |=
          static_cast<uint16_t>(1U << index);
      observed = get_filter_values(
          radar_filter_configs_[sensor_id], filterTypes[index]);
    }

    {
      std::lock_guard<std::mutex> confirmation_lock(filter_confirmation_mutex_);
      auto &pending = pending_filter_confirmation_;
      if (!pending.waiting ||
          pending.local_index != static_cast<std::size_t>(sensor_id) ||
          pending.type != type ||
          pending.index != index)
      {
        return;
      }
      const bool reply_is_fresh =
          frame->has_valid_bus_timestamp()
              ? frame->get_bus_time() >= pending.bus_not_before
              : frame->get_receive_time() >= pending.receive_not_before;
      if (!reply_is_fresh)
      {
        return;
      }

      const auto range = get_filter_range(filterTypes[index]);
      const double tolerance = range.resolution * 0.51 + 1e-6;
      const bool active_matches = observed.active == pending.active;
      const bool limits_match =
          !pending.compare_limits ||
          (std::abs(observed.min_value - pending.min_value) <= tolerance &&
           std::abs(observed.max_value - pending.max_value) <= tolerance);
      if (active_matches && limits_match)
      {
        pending.received = true;
        pending.matched = true;
      }
      else
      {
        // A controller/radar can emit the previous state before the requested
        // state. Record it, but keep waiting until the deadline for a match.
        pending.saw_mismatch = true;
        return;
      }
    }
    filter_confirmation_cv_.notify_all();
  }

  void radar_conti_srr308::publishFilterConfigMetadata()
  {
    std::vector<radar_conti_srr308_msgs::msg::FilterStateCfg> filter_configs;
    {
      std::lock_guard<std::mutex> lock(filter_config_mutex_);
      filter_configs = radar_filter_configs_;
    }

    for (size_t sensor_id = 0; sensor_id < filter_configs.size(); ++sensor_id)
    {
      const auto &config = filter_configs[sensor_id];
      if (config.valid_mask == 0U)
      {
        continue;
      }

      filter_config_publishers_[sensor_id]->publish(config);

      constexpr uint16_t distance_bit = static_cast<uint16_t>(1U << 1U);
      constexpr uint16_t azimuth_bit = static_cast<uint16_t>(1U << 2U);
      const bool distance_seen = (config.valid_mask & distance_bit) != 0U;
      const bool azimuth_seen = (config.valid_mask & azimuth_bit) != 0U;
      const bool either_observed_inactive =
          (distance_seen && !config.distance.active) ||
          (azimuth_seen && !config.azimuth.active);

      if (distance_seen && azimuth_seen &&
          config.distance.active && config.azimuth.active)
      {
        auto fov_filter_markers = radar_visualization::createFilteredRadarFOVMarker(
            radar_link_names_[sensor_id], config.azimuth.min, config.azimuth.max,
            config.distance.min, config.distance.max, this->get_clock());
        fov_filter_marker_publishers_[sensor_id]->publish(fov_filter_markers);
      }
      else if (either_observed_inactive)
      {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = radar_link_names_[sensor_id];
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.ns = "radar_fov";
        delete_marker.id = 0;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        delete_marker.pose.orientation.w = 1.0;
        fov_filter_marker_publishers_[sensor_id]->publish(delete_marker);
      }
    }
  }

  void radar_conti_srr308::publishFilteredFovDeleteMarkers()
  {
    for (std::size_t sensor_id = 0;
         sensor_id < fov_filter_marker_publishers_.size() &&
         sensor_id < radar_link_names_.size();
         ++sensor_id)
    {
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.header.frame_id = radar_link_names_[sensor_id];
      delete_marker.header.stamp = this->get_clock()->now();
      delete_marker.ns = "radar_fov";
      delete_marker.id = 0;
      delete_marker.action = visualization_msgs::msg::Marker::DELETE;
      delete_marker.pose.orientation.w = 1.0;
      fov_filter_marker_publishers_[sensor_id]->publish(delete_marker);
    }
  }

  void radar_conti_srr308::resetFilterObservations()
  {
    std::lock_guard<std::mutex> lock(filter_config_mutex_);
    for (auto &config : radar_filter_configs_)
    {
      config = radar_conti_srr308_msgs::msg::FilterStateCfg{};
    }
  }

  void radar_conti_srr308::publishClearedFilterObservations()
  {
    for (std::size_t sensor_id = 0;
         sensor_id < filter_config_publishers_.size() &&
         sensor_id < radar_link_names_.size();
         ++sensor_id)
    {
      radar_conti_srr308_msgs::msg::FilterStateCfg cleared;
      cleared.header.stamp = this->get_clock()->now();
      cleared.header.frame_id = radar_link_names_[sensor_id];
      cleared.valid_mask = 0U;
      // This explicit zero-mask sample replaces any TransientLocal sample
      // cached during the previous SocketCAN receive session.
      filter_config_publishers_[sensor_id]->publish(cleared);
    }
  }

  void radar_conti_srr308::publishFovMetadata()
  {
    for (size_t sensor_id = 0; sensor_id < radar_link_names_.size(); ++sensor_id)
    {
      auto fov_markers = radar_visualization::createRadarFOVMarkers(
          radar_link_names_[sensor_id],
          this->get_clock());
      fov_marker_publishers_[sensor_id]->publish(fov_markers);
    }
  }

  void radar_conti_srr308::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    vehicle_odometry_ = *msg;

    for (auto &motion_config : motion_configs_)
    {
      auto local_index = motion_config.first;
      auto enable = motion_config.second;
      if (enable)
      {
        if (!enable_can_tx_)
        {
          RCLCPP_DEBUG_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              1000,
              "Skipping motion input CAN write because enable_can_tx is false.");
          continue;
        }
        auto motion_input_signal = radar_transforms::createMotionInputSignal(vehicle_odometry_, tf_buffer_, radar_link_names_[local_index], robot_base_frame_, transform_timeout_, this->get_clock());
        sendMotionInputSignals(radar_sensor_ids_[local_index], motion_input_signal);
      }
    }
  }

  void radar_conti_srr308::sendMotionInputSignals(const size_t &sensor_id, radar_conti_srr308_structs::MotionInputSignal &motion_input_signal)
  {
    if (!enable_can_tx_)
    {
      return;
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sending motion input");
    // Set up speed frame
    polymath::socketcan::CanFrame speed_frame;
    speed_frame.set_len(DLC_SpeedInformation);
    uint32_t speed_msg_id = ID_SpeedInformation;
    Set_SensorID_In_MsgID(speed_msg_id, sensor_id);
    speed_frame.set_can_id(speed_msg_id);
    std::array<unsigned char, CAN_MAX_DLC> speed_data{0};

    SET_SpeedInformation_RadarDevice_Speed(speed_data, motion_input_signal.speed / SpeedInformation::SPEED_RESOLUTION);
    // TODO(troy): Use odom tf to see if the radar is mounted backwards or forwards
    SET_SpeedInformation_RadarDevice_SpeedDirection(speed_data, motion_input_signal.direction);

    speed_frame.set_data(speed_data);
    auto speed_err = socketcan_adapter_->send(speed_frame);
    if (speed_err.has_value())
    {
      auto msg = fmt::format("Error sending speed frame: {}", speed_err.value());
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
    }

    // Set up yaw rate frame
    polymath::socketcan::CanFrame yaw_rate_frame;
    yaw_rate_frame.set_len(DLC_YawRateInformation);
    uint32_t yaw_rate_msg_id = ID_YawRateInformation;
    Set_SensorID_In_MsgID(yaw_rate_msg_id, sensor_id);
    yaw_rate_frame.set_can_id(yaw_rate_msg_id);
    std::array<unsigned char, CAN_MAX_DLC> yaw_rate_info_data{0};

    SET_YawRateInformation_RadarDevice_YawRate(yaw_rate_info_data, (motion_input_signal.yaw_rate + YawRateInformation::YAW_RATE_OFFSET) / YawRateInformation::YAW_RATE_RESOLUTION);

    yaw_rate_frame.set_data(yaw_rate_info_data);
    auto yaw_err = socketcan_adapter_->send(yaw_rate_frame);
    if (yaw_err.has_value())
    {
      auto msg = fmt::format("Error sending yaw frame: {}", yaw_err.value());
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
    }
  }

} // end namespace

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// CLASS_LOADER_REGISTER_CLASS(FHAC::radar_conti_srr308, rclcpp_lifecycle::LifecycleNode)

RCLCPP_COMPONENTS_REGISTER_NODE(FHAC::radar_conti_srr308)
