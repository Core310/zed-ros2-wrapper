# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from math import pi

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command
)
from launch.actions import (
    DeclareLaunchArgument
)
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)

# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

# Cmaera name and model
camera_name = 'zed'
camera_model = 'zed2i'

# RVIZ2 Configurations to be loaded by ZED Node
config_rviz2 = os.path.join(
    get_package_share_directory('zed_robot_integration'),
    'rviz2','view_config.rviz'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
xacro_path = os.path.join(
    get_package_share_directory('zed_robot_integration'),
    'urdf',
    'zed_robot_mono.urdf.xacro'
)

def launch_setup(context, *args, **kwargs):
     # Launch configuration variables
    use_zed_localization = LaunchConfiguration('use_zed_localization')


    # Robot URDF from xacro
    robot_description = Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name, ' ',
                    'camera_model:=', camera_model, ' ',
                    'use_zed_localization:=', use_zed_localization, 
                ])

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )
    # node that converts 3d -> 2d laser scans (we dont use/need it)
    pcl_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', "/unilidar/cloud"),
                    ('scan', "/scan")],
        parameters=[{
            'transform_tolerance': 0.05,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -pi,
            'angle_max': pi,
            'angle_increment': pi / 180.0 / 2.0,
            'scan_time': 1 / 10,  # 10Hz
            'range_min': 0.1,
            'range_max': 100.0,
            'use_inf': True,
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',  # x y z
            '0', '0', '0', '1',  # quaternion rot
            'base_link',  # parent
            'unilidar_link'  # child (the frame stamped on your PointCloud2)
        ]
    )


# Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='scoutm_robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ])

    # Joint State Publisher
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='scoutm_joint_state_publisher'
    )

    return [
        rviz2_node,
        rsp_node,
        jsp_node,
        pcl_node,
        static_tf
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_zed_localization',
                default_value='true',
                description='Creates a TF tree with `camera_link` as root frame if `true`, otherwise the root is `base_link`.',
                choices=['true', 'false']),
            OpaqueFunction(function=launch_setup)    
        ]
    )