#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mrs_uav_trajectory_loader')

    # Declare arguments
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('uav_name', default_value='uav1', description='UAV namespace'))
    ld.add_action(DeclareLaunchArgument('mode', default_value='load', description='Operation mode: load/goto/track/stop'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info', description='Log level'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'))

    uav_name = LaunchConfiguration('uav_name')
    mode_arg = LaunchConfiguration('mode')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Config file
    default_cfg = PathJoinSubstitution([pkg_share, 'config', 'my_example.yaml'])

    # Composable node
    node = ComposableNode(
        package='mrs_uav_trajectory_loader',
        plugin='mrs_uav_trajectory_loader::TrajectoryLoaderNode',
        name='trajectory_loader',
        namespace=uav_name,
        parameters=[
            default_cfg,
            {'trajectory.mode': mode_arg},
            {'use_sim_time': use_sim_time},
        ],
    )

    container = ComposableNodeContainer(
        name=[uav_name, '_trajectory_loader_container'],
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[node],
    )

    ld.add_action(container)
    return ld