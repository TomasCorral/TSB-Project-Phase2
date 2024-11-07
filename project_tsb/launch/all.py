import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

package_name = 'project_tsb'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=package_name,
            executable='simulator',
            name='simulator',
            output='log',
            parameters=[
                {"deltat": 0.2},
                {"initial_x": 0.0},
                {"initial_y": 0.0},
                {"initial_yaw": 0.0},
                {"initial_u": 0.0},
                {"initial_v": 0.0},
                {"initial_r": 0.0},
                {"current_limit": 20.0},
                {"motor_deadzone": 0.5}
            ]
        ),
        Node(
            package=package_name,
            executable='pid',
            name='pid',
            output='log',
            parameters=[
                {"deltat": 0.5},
                {"kp_u": 30.0}, # 50.0
                {"ki_u": 17.0}, # 18.0
                {"kd_u": 1.0}, # 5.0
                {"kp_yaw": 7.5}, # 6.5
                {"ki_yaw": 0.0}, # 0.0
                {"kd_yaw": 9.0} # 8.0
            ]
        ),
        Node(
            package=package_name,
            executable='motorcontroller',
            output='log',
            name='motorcontroller',
        ),
        Node(
            package=package_name,
            executable='pathfollower',
            name='pathfollower',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        )
    ])