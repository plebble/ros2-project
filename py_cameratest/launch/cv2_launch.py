import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='py_cameratest',
            executable='talker',
            name='talker1'
        ),
        Node(
            package='py_cameratest',
            executable='listener',
            name='listener1'
        )

  ])