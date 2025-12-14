"""
Launch file for autonomous navigation system
Starts both obstacle_analyzer and velocity_controller nodes
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # TODO: Ajouter le nœud obstacle_analyzer
        # Exemple de structure:
        Node(
            package='turtlebot3_controller',
            executable='obstacle_analyzer',
            name='obstacle_analyzer',
            output='screen',
            parameters=[{
                'danger_threshold': 0.5
            }]
        ),
        
        # TODO: Ajouter le nœud velocity_controller
        # Avec les paramètres:
        #   - max_linear_speed: 0.22
        #   - max_angular_speed: 1.5
        #   - safe_distance: 0.5
        #   - comfort_distance: 0.8

        Node(
            package='turtlebot3_controller',
            executable='velocity_controller',
            name='velocity_controller',
            output='screen',
            parameters=[{
                'max_linear_speed': 0.22,
                'max_angular_speed: 1.5': 1.5,
                'safe_distance': 0.5,
                'comfort_distance': 0.8,
            }]
        ),
    ])
