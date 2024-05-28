import launch
import launch_ros.actions
from launch.actions import TimerAction

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_py_package',
            executable='cam_to_num_subcar',
            name='cam_to_num'
        ),
        launch_ros.actions.Node(
            package='my_py_package',
            executable='speed_subcar',
            name='speed'
        ),
        launch_ros.actions.Node(
            package='my_py_package',
            executable='steering_subcar',
            name='steering'
        ),
        launch_ros.actions.Node(
            package='my_py_package',
            executable='distance',
            name='distance'
        ),
    ])