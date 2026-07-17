#ifndef COMPOSITION__RADAR_CONTI_SRR308_COMPONENT_HPP_
#define COMPOSITION__RADAR_CONTI_SRR308_COMPONENT_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <can_msgs/msg/frame.hpp>
#include <radar_conti_srr308_msgs/msg/object_list.hpp>
#include <radar_conti_srr308_msgs/msg/cluster_list.hpp>
#include <radar_conti_srr308_msgs/msg/radar_state.hpp>
#include <radar_conti_srr308_msgs/msg/cluster_status.hpp>
#include "radar_conti_srr308_msgs/msg/filter_state_cfg.hpp"
#include "radar_conti_srr308_msgs/msg/radar_configuration.hpp"
#include "radar_conti_srr308_msgs/msg/radar_state.hpp"
#include <radar_conti_srr308_msgs/srv/set_filter.hpp>
#include <radar_conti_srr308_msgs/srv/trigger_set_cfg.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rmw/qos_profiles.h"

#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "radar_msgs/msg/radar_track.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"

#include "nav2_dynamic_msgs/msg/obstacle.hpp"
#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

#include <srr308_can_defines.h>
#include <structs.hpp>

#include "unique_identifier_msgs/msg/uuid.hpp"

#include <random>

// Enum class definition
enum class FilterType
{
    NOFOBJ,
    DISTANCE,
    AZIMUTH,
    VRELONCOME,
    VRELDEPART,
    RCS,
    LIFETIME,
    SIZE,
    PROBEXISTS,
    Y,
    X,
    VYRIGHTLEFT,
    VXONCOME,
    VYLEFTRIGHT,
    VXDEPART,
    UNKNOWN // Add this to handle default case
};

constexpr char DEFAULT_NODE_NAME[] = "RADAR_CONTI_SRR308";

typedef unsigned char ubyte;
typedef unsigned short int uword;

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

namespace FHAC
{

    class radar_conti_srr308 : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        RADAR_CONTI_SRR308_PUBLIC
        radar_conti_srr308(const rclcpp::NodeOptions &options);
        ~radar_conti_srr308() override;

        /// Transition callback for state error
        /**
         * on_error callback is being called when the lifecycle node
         * enters the "error" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &
                previous_state);

        /// Transition callback for state shutting down
        /**
         * on_shutdown callback is being called when the lifecycle node
         * enters the "shutting down" state.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state);

        /// Transition callback for state configuring
        /**
         * on_configure callback is being called when the lifecycle node
         * enters the "configuring" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "unconfigured".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &);

        /// Transition callback for state activating
        /**
         * on_activate callback is being called when the lifecycle node
         * enters the "activating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "active" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "active"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         *
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &);

        /// Transition callback for state deactivating
        /**
         * on_deactivate callback is being called when the lifecycle node
         * enters the "deactivating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "active".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "active"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &);

        /// Transition callback for state cleaningup
        /**
         * on_cleanup callback is being called when the lifecycle node
         * enters the "cleaningup" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "unconfigured" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &);

        void setFilterService(
            const std::shared_ptr<radar_conti_srr308_msgs::srv::SetFilter::Request> request,
            std::shared_ptr<radar_conti_srr308_msgs::srv::SetFilter::Response> response);

        void setRadarConfigurationService(
            const std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Request> request,
            std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Response> response);

        void setFilterConfigurationService(
            const std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Request> request,
            std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Response> response);

        template <typename T>
        void declare_parameter_with_type(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string &param_name, T value)
        {
            if (node->has_parameter(param_name))
            {
                return;
            }
            if constexpr (std::is_same_v<T, int>)
            {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<int>(value)));
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<double>(value)));
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<float>(value)));
            }
            else if constexpr (std::is_same_v<T, uint32_t>)
            {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<int>(value)));
            }
            else if constexpr (std::is_same_v<T, uint8_t>)
            {
                node->declare_parameter(param_name, rclcpp::ParameterValue(static_cast<int>(value)));
            }
            else
            {
                static_assert(!std::is_same_v<T, T>, "Unsupported type for declare_parameter_with_type");
            }
        }

        // Function to handle the retrieval of parameters based on type T
        template <typename T>
        void get_parameter_with_type(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string &param_name, T &value)
        {
            if constexpr (std::is_same_v<T, int>)
            {
                node->get_parameter(param_name, value);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                node->get_parameter(param_name, value);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                node->get_parameter(param_name, value);
            }
            else if constexpr (std::is_same_v<T, uint32_t>)
            {
                int temp_value;
                node->get_parameter(param_name, temp_value);
                value = static_cast<uint32_t>(temp_value);
            }
            else if constexpr (std::is_same_v<T, uint8_t>)
            {
                int temp_value;
                node->get_parameter(param_name, temp_value);
                if (temp_value < 0 || temp_value > 255)
                {
                    throw std::out_of_range(param_name + " must be in [0, 255]");
                }
                value = static_cast<uint8_t>(temp_value);
            }
            else
            {
                static_assert(!std::is_same_v<T, T>, "Unsupported type for get_parameter_with_type");
            }
        }

        template <typename T>
        void initializeConfig(std::string radar_name, std::string config_name, T value, T &config);

        unique_identifier_msgs::msg::UUID generateRandomUUID();
        void generateUUIDTable();

        static constexpr double covariance[] = {
            0.005,
            0.007,
            0.010,
            0.014,
            0.020,
            0.029,
            0.041,
            0.058,
            0.082,
            0.116,
            0.165,
            0.234,
            0.332,
            0.471,
            0.669,
            0.949,
            1.346,
            1.909,
            2.709,
            3.843,
            5.451,
            7.734,
            10.971,
            15.565,
            22.081,
            31.325,
            44.439,
            63.044,
            89.437,
            126.881,
            180.000,
            200.000};

    private:
        // ##############Task2################
        std::unique_ptr<polymath::socketcan::SocketcanAdapter> socketcan_adapter_;
        // create Publisher
        rclcpp::QoS qos{10};
        std::string object_list_topic_name_;
        // SRR308 Change
        std::string cluster_list_topic_name_;
        // *********************************
        std::string marker_array_topic_name_;
        std::string radar_tracks_topic_name_;
        std::string obstacle_array_topic_name_;
        std::string filter_config_topic_name_;
        std::string radar_state_topic_name_;
        std::string pub_tf_topic_name = "tf";
        std::string radar_link_;

        uint16_t max_radar_id = 512;
        std::vector<unique_identifier_msgs::msg::UUID> UUID_table_;

        // Publishers
        std::vector<rclcpp_lifecycle::LifecyclePublisher<radar_conti_srr308_msgs::msg::ObjectList>::SharedPtr> object_list_publishers_;
        // SRR308 Change
        std::vector<rclcpp_lifecycle::LifecyclePublisher<radar_conti_srr308_msgs::msg::ClusterList>::SharedPtr> cluster_list_publishers_;
        // *****************************************************************
        std::vector<rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>::SharedPtr> tf_publishers_;
        std::vector<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr> marker_array_publishers_;
        std::vector<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr> fov_marker_publishers_;
        std::vector<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr> fov_filter_marker_publishers_;
        std::vector<rclcpp_lifecycle::LifecyclePublisher<radar_msgs::msg::RadarTracks>::SharedPtr> radar_tracks_publishers_;
        std::vector<rclcpp_lifecycle::LifecyclePublisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr> obstacle_array_publishers_;
        std::vector<rclcpp_lifecycle::LifecyclePublisher<radar_conti_srr308_msgs::msg::FilterStateCfg>::SharedPtr> filter_config_publishers_;
        std::vector<rclcpp_lifecycle::LifecyclePublisher<radar_conti_srr308_msgs::msg::RadarState>::SharedPtr> radar_state_publishers_;

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;

        // Services
        rclcpp::Service<radar_conti_srr308_msgs::srv::SetFilter>::SharedPtr set_filter_service_;
        rclcpp::Service<radar_conti_srr308_msgs::srv::TriggerSetCfg>::SharedPtr filter_config_service_;
        rclcpp::Service<radar_conti_srr308_msgs::srv::TriggerSetCfg>::SharedPtr radar_config_service_;

        // create can_receive_callback
        void can_receive_callback(std::shared_ptr<const polymath::socketcan::CanFrame> frame);
        // create handle_object_list
        void handle_object_list(std::shared_ptr<const polymath::socketcan::CanFrame> frame);
        // SRR308 Changes: add cluter mode
        void handle_cluster_list(std::shared_ptr<const polymath::socketcan::CanFrame> frame);
        // create publish_object_map
        void publish_object_map(int sensor_id);
        // Publish and clear one radar's pending cluster scan. The caller must
        // hold cluster_scan_mutex_.
        void publish_cluster_scan(std::size_t local_index);
        void check_cluster_scan_timeouts();
        void flush_pending_cluster_scans();
        rclcpp::Time frame_bus_time(
            const polymath::socketcan::CanFrame &frame) const;
        struct FilterCommandResult
        {
            bool sent{false};
            bool confirmed{false};
            std::string message;
        };

        // Send one Cluster FilterCfg frame and wait for its FilterState_Cfg
        // acknowledgement. The caller serializes configuration writes with
        // can_tx_mutex_.
        FilterCommandResult setFilter(
            int sensor_id, bool active, uint8_t type, uint8_t index,
            double min_value, double max_value);
        bool applyFilterConfiguration(
            std::size_t local_index, std::string &message);
        // update config
        bool setRadarConfiguration(const int &sensor_id, std::shared_ptr<radar_conti_srr308_msgs::srv::TriggerSetCfg::Response> &response);

        void publishRadarState(std::shared_ptr<const polymath::socketcan::CanFrame> frame, const int &sensor_id);
        void updateFilterConfig(std::shared_ptr<const polymath::socketcan::CanFrame> frame, const int &sensor_id);
        void initializeFilterConfigs();
        void startMetadataTimers();
        void publishFilterConfigMetadata();
        void publishFilteredFovDeleteMarkers();
        void publishClearedFilterObservations();
        void resetFilterObservations();
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        nav_msgs::msg::Odometry vehicle_odometry_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Duration transform_timeout_{0, 0};

        void sendMotionInputSignals(const size_t &sensor_id, radar_conti_srr308_structs::MotionInputSignal &motion_input_signal);
        void publishFovMetadata();

        // create map container for object list
        std::map<int, radar_conti_srr308_msgs::msg::Object> object_map_;
        std::vector<std::map<int, radar_conti_srr308_msgs::msg::Object>> object_map_list_;
        struct ClusterScanState
        {
            bool active{false};
            bool status_valid{false};
            uint16_t measurement_counter{0};
            uint8_t interface_version{0};
            uint16_t expected_near_count{0};
            uint16_t expected_far_count{0};
            rclcpp::Time scan_stamp{0, 0, RCL_SYSTEM_TIME};
            bool scan_stamp_valid{false};
            bool all_rx_timestamps_valid{true};
            rclcpp::Time first_rx_stamp{0, 0, RCL_SYSTEM_TIME};
            rclcpp::Time last_rx_stamp{0, 0, RCL_SYSTEM_TIME};
            std::chrono::steady_clock::time_point last_receive_time{};
            std::vector<radar_conti_srr308_msgs::msg::Cluster> clusters;
            std::unordered_set<int> unique_cluster_ids;
        };

        std::vector<ClusterScanState> cluster_scans_;
        std::mutex cluster_scan_mutex_;
        std::chrono::milliseconds cluster_scan_timeout_{200};
        std::size_t max_cluster_frames_per_scan_{512};
        rclcpp::TimerBase::SharedPtr cluster_scan_timeout_timer_;

        // create data structures for radar object list
        radar_conti_srr308_msgs::msg::ObjectList object_list_;
        std::vector<radar_conti_srr308_msgs::msg::ObjectList> object_list_list_;
        
        // create data structures for radar filter config
        rclcpp::TimerBase::SharedPtr filter_config_timer_;
        std::mutex filter_config_mutex_;
        // Desired values loaded from YAML are kept separate from values
        // observed in FilterState_Cfg replies.
        std::vector<radar_conti_srr308_msgs::msg::FilterStateCfg> desired_filter_configs_;
        std::vector<radar_conti_srr308_msgs::msg::FilterStateCfg> radar_filter_configs_;
        std::vector<radar_conti_srr308_msgs::msg::RadarConfiguration> radar_configuration_configs_;
        std::unordered_map<size_t, bool> motion_configs_;
        std::vector<std::vector<bool>> radar_filter_active_;
        std::vector<int> radar_sensor_ids_;
        std::unordered_map<int, size_t> sensor_id_to_local_index_;

        struct PendingFilterConfirmation
        {
            bool waiting{false};
            bool received{false};
            bool matched{false};
            bool saw_mismatch{false};
            bool cancelled{false};
            std::size_t local_index{0};
            uint8_t type{0};
            uint8_t index{0};
            bool active{false};
            bool compare_limits{false};
            double min_value{0.0};
            double max_value{0.0};
            // Kernel CAN timestamps are compared against bus_not_before so a
            // FilterState frame queued before this request cannot confirm it.
            std::chrono::system_clock::time_point bus_not_before{};
            std::chrono::steady_clock::time_point receive_not_before{};
        };
        std::mutex can_tx_mutex_;
        std::mutex filter_confirmation_mutex_;
        std::condition_variable filter_confirmation_cv_;
        PendingFilterConfirmation pending_filter_confirmation_;
        std::chrono::milliseconds filter_confirmation_timeout_{300};

        // additional variables
        rclcpp::TimerBase::SharedPtr fov_marker_timer_;
        int object_count;
        int number_of_radars_;
        std::string can_channel_;
        std::string odom_topic_name_;

        /// @brief whether to overwrite radar configurations on startup
        bool overwrite_configs_;
        bool enable_can_tx_;
        bool enable_manual_can_tx_;
        double cluster_marker_lifetime_sec_;
        double cluster_marker_scale_;

        std::vector<std::string> radar_link_names_;
        std::string robot_base_frame_;

        // ##################################
    };

} // namespace radar_conti_srr308

#endif // COMPOSITION__RADAR_CONTI_SRR308_COMPONENT_HPP_
