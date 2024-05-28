#!/usr/bin/env python3
import launch
import launch_ros.actions
from launch.actions import TimerAction

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_py_package',
            executable='distance',
            name='distance'
        ),
        TimerAction(
            period=2.0,  # 2초 지연
            actions=[
                launch_ros.actions.Node(
                    package='my_py_package',
                    executable='cam_to_num',
                    name='cam_to_num'
                )
            ]
        ),
        TimerAction(
            period=2.0,  # 2초 지연
            actions=[
                launch_ros.actions.Node(
                    package='my_py_package',
                    executable='steering',
                    name='steering'
                )
            ]
        ),
        TimerAction(
            period=3.0,  # 2초 지연
            actions=[
                launch_ros.actions.Node(
                    package='my_py_package',
                    executable='speed2',
                    name='speed2'
                )
            ]
        ),
        
    ])