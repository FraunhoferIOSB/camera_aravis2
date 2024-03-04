import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

# Start as node:

def generate_launch_description():
    
    example_package_node = Node(
        name="camera_aravis_example",
        package="camera_aravis",
        executable="camera_aravis",
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
                {
                "guid": "Allied Vision-Alvium G1-240c-05P3C",
                "stream_names": ["vis"],
                "pixel_formats": ["RGB8"],
                "camera_info_urls": [os.path.join(
                    get_package_share_directory('camera_aravis'),
                    'config/camera_info_example.yaml')]
                }
            ]
    )
    return launch.LaunchDescription([example_package_node])
