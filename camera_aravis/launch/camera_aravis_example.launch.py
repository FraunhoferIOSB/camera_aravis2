# Copyright (c) 2024 Fraunhofer IOSB and contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} '
# '({function_name}() at {file_name}:{line_number})'


# Start as node:
def generate_launch_description():

    example_package_node = Node(
        name="camera_aravis2_example",
        package="camera_aravis2",
        executable="camera_aravis2",
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
                {
                    "guid": "Allied Vision-Alvium G1-240c-05P3C",
                    "stream_names": ["vis"],
                    "pixel_formats": ["BayerRG8"],
                    "camera_info_urls": ["file://" + os.path.join(
                        get_package_share_directory('camera_aravis2'),
                        'config/camera_info_example.yaml')],
                    "frame_rate": 5.0
                }
            ]
    )
    return launch.LaunchDescription([example_package_node])
