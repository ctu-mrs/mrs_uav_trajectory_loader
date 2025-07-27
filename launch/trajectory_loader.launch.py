#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ---- Launch args ----
    default_uav = os.getenv("UAV_NAME", "uav1")
    ld_uav  = DeclareLaunchArgument("uav_name",  default_value=default_uav,  description="UAV name (namespace).")
    ld_log  = DeclareLaunchArgument("log_level", default_value="info")
    ld_sim  = DeclareLaunchArgument("use_sim_time", default_value=os.getenv("USE_SIM_TIME", "false"))

    uav_name    = LaunchConfiguration("uav_name")
    log_level   = LaunchConfiguration("log_level")
    use_sim_time= LaunchConfiguration("use_sim_time")

    # ---- Default config file ----
    pkg_share = get_package_share_directory("mrs_uav_trajectory_loader")
    default_cfg = PathJoinSubstitution([pkg_share, "config", "my_example.yaml"])

    # (опционально) сформировать имя сервиса прямо здесь
    load_name_param = {
        "service": {
            "load_name": ["/", uav_name, "/control_manager/trajectory_reference"]
        }
    }

    node = ComposableNode(
        package="mrs_uav_trajectory_loader",
        plugin="mrs_uav_trajectory_loader::TrajectoryLoaderNode",
        name="trajectory_loader",
        parameters=[
            default_cfg,
            {"use_sim_time": use_sim_time},
            load_name_param,   # можно убрать, если задаёшь в yaml
        ],
    )

    container = ComposableNodeContainer(
        namespace=uav_name,
        name="trajectory_loader_container",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=[node],
    )

    return LaunchDescription([ld_uav, ld_log, ld_sim, container])
