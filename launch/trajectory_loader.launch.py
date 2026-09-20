#!/usr/bin/env python3

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    pkg_name = "mrs_multi_uav_trajectory_loader"
    pkg_share = get_package_share_directory(pkg_name)

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------

    # uav_name
    uav_name = LaunchConfiguration("uav_name")

    ld.add_action(DeclareLaunchArgument(
            name="uav_name",
            default_value=os.getenv("UAV_NAME", ""),
            description=(
                "UAV name where the loader is running. "
                "Can be empty when running as a centralized loader."
            ),
    ))


    # config
    default_config = PathJoinSubstitution([pkg_share, "config", "default.yaml"])

    ld.add_action(
        DeclareLaunchArgument(
            name="config",
            default_value=default_config,
            description="Path or filename for multi-UAV trajectory configuration file.",
        )
    )

    config_arg = LaunchConfiguration("config")

    config = IfElseSubstitution(
        condition=PythonExpression(["'", config_arg, "'.startswith('/')"]),
        if_value=config_arg,
        else_value=PathJoinSubstitution([pkg_share, "config", config_arg]),
    )

    # service config
    service_default_config = PathJoinSubstitution([pkg_share, "config", "services.yaml"])

    ld.add_action(
        DeclareLaunchArgument(
            name="service_config",
            default_value=service_default_config,
            description="Path or filename for multi-UAV trajectory configuration file.",
        )
    )

    service_config_arg = LaunchConfiguration("service_config")

    service_config = IfElseSubstitution(
        condition=PythonExpression(["'", service_config_arg, "'.startswith('/')"]),
        if_value=service_config_arg,
        else_value=PathJoinSubstitution([pkg_share, "config", service_config_arg]),
    )

    # use_sim_time
    use_sim_time = LaunchConfiguration("use_sim_time")
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value=os.getenv("USE_SIM_TIME", "false"),
        description="Use simulation time.",
    ))

    # log_level
    log_level = LaunchConfiguration("log_level")
    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level.",
    ))

    # debug
    debug = LaunchConfiguration("debug")
    ld.add_action(
        DeclareLaunchArgument(
            name="debug",
            default_value="false",
            description="Runs the node within a gdb debug session.",
        )
    )

    debug = IfElseSubstitution(
        condition=PythonExpression(['"', debug, '" == "true"']),
        if_value="debug_roslaunch " + os.ttyname(sys.stdout.fileno()),
        else_value="",
    )

    # trajectory path
    pkg_config_path = os.path.join(pkg_share, 'config')


    # --------------------------------------------------------------------------
    # Node description
    # --------------------------------------------------------------------------

    node = Node(
        package=pkg_name,
        executable="trajectory_loader_node",
        name="trajectory_loader",
        namespace=uav_name,
        prefix=[debug],
        output="screen",
        parameters=[
            {"uav_name": uav_name,
             "config": config,
             "config_dir_path": pkg_config_path,
             "service_config": service_config,
             "use_sim_time": use_sim_time,
            },
        ],

        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
    )

    ld.add_action(node)
    return ld