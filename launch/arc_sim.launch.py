from launch import LaunchDescription
from launch import actions
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arc_detection',
            executable='adc_model',
            on_exit=[actions.Shutdown()],
            ros_arguments=[
                '--log-level', 'adc_model:=info'
            ],
        ),
        Node(
            package='arc_detection',
            executable='dma_model',
            on_exit=[actions.Shutdown()],
            ros_arguments=[
                '--log-level', 'dma_model:=info'
            ],
        ),
        Node(
            package='arc_detection',
            executable='detection_model',
            ros_arguments=[
                '--log-level', 'detection_model:=info'
            ],
        )
    ])