from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'joy_linux',
            executable = 'joy_linux_node',
            name = 'joystick_node',
            parameters= [{
                "dev": "/dev/input/js4"
            }]
        ),
        Node(
            package = 'teleop_twist_joy',
            executable = 'teleop_node',
            parameters = [{
                "require_enable_button": False,
                "axis_linear": {"x": 1, "y": 0},
                "axis_angular": {"yaw": 2},
                "scale_linear": {"x": 0.5, "y": 0.5},
                "scale_angular": {"yaw": 7.5}
                }]
        )
    ])