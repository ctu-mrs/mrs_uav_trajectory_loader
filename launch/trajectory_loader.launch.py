#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()
    namespace = "trajectory_loader"

    # Declare arguments
    uav_name = LaunchConfiguration("uav_name")

    ld.add_action(
        DeclareLaunchArgument(
            name="uav_name",
            default_value=os.getenv("UAV_NAME", "uav1"),
            description="The uav name used for namespacing.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="log_level", default_value="info", description="Log level"
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    ld.add_action(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value=os.getenv("USE_SIM_TIME", "false"),
            description="Should the node subscribe to sim time?",
        )
    )

    custom_config = LaunchConfiguration("custom_config")

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(
        DeclareLaunchArgument(
            name="custom_config",
            default_value="",
            description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        )
    )

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(
            [
                '"',
                custom_config,
                '" != "" and ',
                'not "',
                custom_config,
                '".startswith("/")',
            ]
        ),
        if_value=PathJoinSubstitution([EnvironmentVariable("PWD"), custom_config]),
        else_value=custom_config,
    )

    # Composable node
    node = ComposableNode(
        package="mrs_uav_trajectory_loader",
        plugin="mrs_uav_trajectory_loader::TrajectoryLoader",
        name="trajectory_loader",
        namespace=uav_name,
        parameters=[
            {"uav_name": uav_name},
            {"use_sim_time": use_sim_time},
            {"custom_config": custom_config},
        ],
    )

    container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace + "_container",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[node],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    ld.add_action(container)
    return ld
