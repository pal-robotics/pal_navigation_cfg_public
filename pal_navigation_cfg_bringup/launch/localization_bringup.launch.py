# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    map_yaml_file = LaunchConfiguration("map")
    configured_params = LaunchConfiguration("params_file")

    nav2_bringup_pkg = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )
    pmb2_maps_dir = get_package_share_directory("pmb2_maps")

    declare_params_file_launch_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("pal_navigation_cfg_params"),
            "localization",
            "localization.yaml",
        ),
        description=(
            "Full path to the ROS2 parameters file to use for Localization"
            " with AMCL configuration"
        ),
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pmb2_maps_dir, "config", "map.yaml"),
        description="Full path to map yaml file to load",
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_pkg, "/localization_launch.py"]
        ),
        launch_arguments={
            "params_file": configured_params,
            "map": map_yaml_file,
        }.items(),
    )

    cmd_vel_remap = SetRemap(src="cmd_vel", dst="nav_vel")

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_pkg, "/navigation_launch.py"]
        ),
        launch_arguments={"params_file": configured_params}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_pkg, "/rviz_launch.py"])
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_params_file_launch_arg)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(localization_launch)
    ld.add_action(cmd_vel_remap)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_launch)

    return ld
