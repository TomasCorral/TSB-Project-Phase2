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
                {"deltat": 0.1}, # Update rate of the simulator (seconds)
                {"initial_x": 0.0}, # Initial x position of the boat (m)
                {"initial_y": 0.0}, # Initial y position of the boat (m)
                {"initial_yaw": 0.0}, # Initial yaw of the boat (degrees)
                {"initial_u": 0.0}, # Initial surge speed of the boat (m/s)
                {"initial_v": 0.0}, # initial sway speed of the boat (m/s)
                {"initial_r": 0.0}, # Initial yaw rate of the boat (deg/s)
                {"current_limit": 20.0}, # Motor current limit (A)
                {"motor_deadzone": 0.2} # Motor deadzone (A)
            ]
        ),
        Node(
            package=package_name,
            executable='pid',
            name='pid',
            parameters=[
                {"deltat": 0.2}, # Update rate of the PID controller (seconds)
                {"reset_integrator_u": False}, # Reset sway integrator value when reference changes
                {"reset_integrator_yaw": False}, # Reset yaw integrator value when reference changes
                {"skip_derivator_u": False}, # Skip first sway derivative update when reference changes
                {"skip_derivator_yaw": True}, # Skip first yaw derivative update when reference changes
                {"acceptable_yaw_error": 5.0} # When speed is 0, yaw error that PID wont change reference 
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
            parameters=[
                {"deltat": 2.0}, # Update rate of the path follower (seconds)
                {"cruising_speed": 0.5}, # Speed of the boat when following the path
                {"minimum_speed": 0.2}, # Minimum speed of the boat
                {"reach_radius": 4.0}, # Reach radius for the rest of the points
                {"reach_radius_last": 1.0}, # Reach radius for the last point
                {"slowdown_distance": 6.0}, # Distance to start slowing down near last point
                {"look_ahead": 3.0}, # Look ahead distance
                {"generate_spline": True}, # Generate spline from path points if true
                {"min_spacing": 10.0} # Minimum distance betwen points when generating path
            ]
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
        )
    ])