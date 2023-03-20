# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from typing import Dict
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch_pal.param_utils import parse_parametric_yaml


def slam_and_nav(context, *args, **kwargs):
    remappings_file = LaunchConfiguration("remappings_file").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)

    nav2_bringup_pkg = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    source_files = []

    with open(remappings_file, "r") as remappings_yaml:
        remappings = yaml.safe_load(remappings_yaml)

    with open(params_file, "r") as params_yaml:
        params: Dict = yaml.safe_load(params_yaml)
        for key, param in params.items():
            source_files.append(
                os.path.join(
                    get_package_share_directory("pal_navigation_cfg_params"),
                    key,
                    param + ".yaml",
                )
            )

    configured_params = parse_parametric_yaml(
        source_files=source_files, param_rewrites=remappings
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_pkg, "/slam_launch.py"]),
        launch_arguments={"params_file": configured_params}.items(),
    )

    cmd_vel_remap = SetRemap(src="cmd_vel", dst="nav_vel")

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_pkg, "/navigation_launch.py"]
        ),
        launch_arguments={"params_file": configured_params}.items(),
    )
    return [slam_toolbox_launch, cmd_vel_remap, nav2_bringup_launch]


def generate_launch_description():
    nav2_bringup_pkg = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    declare_params_file_launch_arg = DeclareLaunchArgument(
        "params_file",
        description=(
            "Full path to the ROS2 parameters file to use for SLAM"
            " configuration"
        ),
    )

    declare_remappings_file_launch_arg = DeclareLaunchArgument(
        "remappings_file",
        default_value=os.path.join(
            get_package_share_directory("pal_navigation_cfg_bringup"),
            "params",
            "default_remappings.yaml",
        ),
        description=(
            "Full path to the ROS2 parameters file to use for NAV2"
            " configuration"
        ),
    )

    slam_and_nav_launch = OpaqueFunction(function=slam_and_nav)

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_pkg, "/rviz_launch.py"])
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_params_file_launch_arg)
    ld.add_action(declare_remappings_file_launch_arg)
    ld.add_action(slam_and_nav_launch)
    ld.add_action(rviz_launch)

    return ld
