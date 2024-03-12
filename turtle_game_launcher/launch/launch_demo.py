from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        Node(
            package='turtle_controller',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='turtle_chaser',
                    executable='turtle_chaser',
                    name='turtle_chaser',
                    output='screen'
                ),
            ]
        ),
    ])
