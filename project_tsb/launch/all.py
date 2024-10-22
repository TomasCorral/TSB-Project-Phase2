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
                {"delta_t": 0.1},
                {"initial_x": 0.0},
                {"initial_y": 0.0},
                {"initial_psi": 0.0},
                {"initial_u": 0.0},
                {"initial_v": 0.0},
                {"initial_r": 0.0}
            ]
        ),
        Node(
            package=package_name,
            executable='pid',
            name='pid',
            parameters=[
                {"delta_t": 2.0},
                {"kp_u": 11.6},
                {"ki_u": 3.0},
                {"kd_u": 0.6},
                {"kp_psi": 0.0},
                {"ki_psi": 0.0},
                {"kd_psi": 0.0}
            ]
        ),
        Node(
            package=package_name,
            executable='plotter.py',
            name='plotter'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'rviz', 'config.rviz')]
        )
    ])