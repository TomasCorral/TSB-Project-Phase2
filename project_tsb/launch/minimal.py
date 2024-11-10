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
                {"deltat": 0.1},
                {"initial_x": 0.0},
                {"initial_y": 0.0},
                {"initial_yaw": 0.0},
                {"initial_u": 0.0},
                {"initial_v": 0.0},
                {"initial_r": 0.0},
                {"current_limit": 20.0},
                {"motor_deadzone": 0.2}
            ]
        ),
        Node(
            package=package_name,
            executable='pid',
            name='pid',
            parameters=[
                {"deltat": 0.2},
                {"reset_integrator_u": False},
                {"reset_integrator_yaw": False},
                {"skip_derivator_u": False},
                {"skip_derivator_yaw": True},
                {"acceptable_yaw_error": 5.0} # Only applies when stoped
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
            parameters=[
                {"deltat": 2.0},
                {"cruising_speed": 1.0}, # Speed of the boat when following the path
                {"minimum_speed": 0.2}, # Minimum speed of the boat
                {"reach_radius": 5.0}, # Reach radius for the rest of the points
                {"reach_radius_last": 1.0}, # Reach radius for the last point
                {"slowdown_distance": 6.0}, # Distance to start slowing down
                {"look_ahead": 1.0} # Look ahead distance
            ]
        )
    ])