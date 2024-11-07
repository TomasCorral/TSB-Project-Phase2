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
            parameters=[
                {"deltat": 0.2},
                {"initial_x": 0.0},
                {"initial_y": 0.0},
                {"initial_yaw": 0.0},
                {"initial_u": 0.0},
                {"initial_v": 0.0},
                {"initial_r": 0.0},
                {"current_limit": 20.0}
            ]
        ),
        Node(
            package=package_name,
            executable='pid',
            name='pid',
            parameters=[
                {"deltat": 1.0},
                {"kp_u": 50.0},
                {"ki_u": 18.0},
                {"kd_u": 5.0},
                {"kp_yaw": 1.4},
                {"ki_yaw": 0.0},
                {"kd_yaw": 7.2}
            ]
        ),
        Node(
            package=package_name,
            executable='motorcontroller',
            name='motorcontroller',
        ),
        Node(
            package=package_name,
            executable='pathfollower',
            name='pathfollower',
        )
    ])