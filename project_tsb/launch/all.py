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
                {"initial_psi": 0.0},
                {"initial_u": 0.0},
                {"initial_v": 0.0},
                {"initial_r": 0.0},
                {"tau_u_limit": 10.0},
                {"tau_r_limit": 5.0} 
            ]
        ),
        Node(
            package=package_name,
            executable='pid',
            name='pid',
            parameters=[
                {"deltat": 1.0},
                {"kp_u": 12.5},
                {"ki_u": 2.8},
                {"kd_u": 2.6},
                {"kp_psi": 0.5},
                {"ki_psi": 0.0},
                {"kd_psi": 3.6}
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