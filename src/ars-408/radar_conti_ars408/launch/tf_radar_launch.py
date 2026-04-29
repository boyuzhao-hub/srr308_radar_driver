# This is a discard file for radar tf launch, which is a radar static trans
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    geometry_param_file = os.path.join(
        get_package_share_directory("radar_conti_ars408"),
        "config",
        "tf_radar_params.yaml"
    )

    with open(geometry_param_file, 'r') as f:
        geometry_params = yaml.safe_load(f)["radar_geometry"]

    front_yaw_deg = geometry_params['front_yaw_deg']
    rear_yaw_deg = geometry_params['rear_yaw_deg']
    height = geometry_params['height']
    front_distance = geometry_params['front_distance']
    rear_distance = geometry_params['rear_distance']

    # --- Translation ---
    front_left_radar_x = front_distance * math.cos(math.radians(front_yaw_deg))
    front_left_radar_y = front_distance * math.sin(math.radians(front_yaw_deg))

    front_right_radar_x = front_distance * math.cos(math.radians(-front_yaw_deg))
    front_right_radar_y = front_distance * math.sin(math.radians(-front_yaw_deg))

    rear_left_radar_x = -rear_distance * math.cos(math.radians(rear_yaw_deg))
    rear_left_radar_y = rear_distance * math.sin(math.radians(rear_yaw_deg))

    rear_right_radar_x = -rear_distance * math.cos(math.radians(-rear_yaw_deg))
    rear_right_radar_y = rear_distance * math.sin(math.radians(-rear_yaw_deg))

    # --- Yaw (transfer to radians) ---
    # front radars Yaw (facing forward)
    yaw_front_left = math.radians(front_yaw_deg)
    yaw_front_right = math.radians(-front_yaw_deg)
    # rear radars Yaw (facing backward)
    yaw_rear_left = math.radians(180.0 - rear_yaw_deg)
    yaw_rear_right = math.radians(-180.0 + rear_yaw_deg)

    # --- Pitch (transfer to radians) ---
    pitch_front = math.radians(geometry_params['front_pitch_deg'])
    pitch_rear = math.radians(geometry_params['rear_pitch_deg'])

    # --- Roll (transfer to radians) ---
    roll_front = math.radians(geometry_params['front_roll_deg'])
    roll_rear = math.radians(geometry_params['rear_roll_deg'])

    return LaunchDescription([
        # front left radar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_front_left_tf',
            arguments=[
                '--x', str(front_left_radar_x), 
                '--y', str(front_left_radar_y), 
                '--z', str(height),
                '--yaw', str(yaw_front_left), 
                '--pitch', str(pitch_front), 
                '--roll', str(roll_front), 
                '--frame-id', 'base_link',
                '--child-frame-id', 'radar_front_left'
            ]
        ), 

        # front right radar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_front_right_tf',
            arguments=[
                '--x', str(front_right_radar_x),
                '--y', str(front_right_radar_y),
                '--z', str(height),
                '--yaw', str(yaw_front_right),
                '--pitch', str(pitch_front),
                '--roll', str(roll_front),
                '--frame-id', 'base_link',
                '--child-frame-id', 'radar_front_right'
            ]
        ), 

        # rear left radar
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='radar_rear_left_tf',
        #     arguments=[
        #         '--x', str(rear_left_radar_x),
        #         '--y', str(rear_left_radar_y),
        #         '--z', str(height),
        #         '--yaw', str(yaw_rear_left),
        #         '--pitch', str(pitch_rear),
        #         '--roll', str(roll_rear),
        #         '--frame-id', 'base_link',
        #         '--child-frame-id', 'radar_rear_left'
        #     ]
        # ),

        # # rear right radar
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='radar_rear_right_tf',
        #     arguments=[
        #         '--x', str(rear_right_radar_x),
        #         '--y', str(rear_right_radar_y),
        #         '--z', str(height),
        #         '--yaw', str(yaw_rear_right),
        #         '--pitch', str(pitch_rear),
        #         '--roll', str(roll_rear),
        #         '--frame-id', 'base_link',
        #         '--child-frame-id', 'radar_rear_right'
        #     ]
        # )
    ])