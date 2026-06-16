# Copyright 2026 Universidad Politecnica de Madrid (UPM).
#
# Author: Guanliang Li
# Contributor: Pedro Espinosa Angulo, Santiago Tapia Fernandez (supervised)
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # detectors_config = os.path.join(get_package_share_directory('sslam_tools'), 'config', 'detectors_config.yaml')
    lidar_config = os.path.join(get_package_share_directory('sslam_tools'), 'config', 'lidar_config.yaml')
    # fusion_config = os.path.join(get_package_share_directory('sslam_tools'), 'config', 'fusion_config.yaml')

    detector_arg = DeclareLaunchArgument(
        'detector_3D',
        default_value = '3D_RANSAC',
        description = 'Select detector. Choices: 3D_RANSAC, 3D_RegionGrow'
    )

    detector_val = LaunchConfiguration('detector_3D')
    
    return LaunchDescription([
        Node(
            package='coord_conversion_cpp',
            executable='converter',
            name='converter',
            parameters=[lidar_config]
        ),
        Node(
            package='sslam_tools',
            executable='detector_node_3D_RANSAC',
            name='detector_node_3D_RANSAC',
            condition=IfCondition(PythonExpression(["'", detector_val, "' == '3D_RANSAC'"]))
        ),
        Node(
            package='sslam_tools',
            executable='detector_node_3D_RegionGrow',
            name='detector_node_3D_RegionGrow',
            condition=IfCondition(PythonExpression(["'", detector_val, "' == '3D_RegionGrow'"]))
        ),
    ])
