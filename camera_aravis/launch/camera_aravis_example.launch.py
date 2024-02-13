import os
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
        parameters=[
            {"guid": "Allied Vision-Alvium G1-240c-05P3C"}
            ]
    )
    return launch.LaunchDescription([example_package_node])
