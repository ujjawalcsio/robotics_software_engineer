import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the URDF file path using the correct path to your URDF
    urdf_file_path = os.path.join(
        '/home/her0/assignment_ws/src/robotics_software_engineer/module_3_assignment', 'urdf', 'wheeled_robot_lift.urdf'
    )

    return LaunchDescription([
        # Declare the launch argument for URDF file
        DeclareLaunchArgument('urdf_file', default_value=urdf_file_path, description='Path to the URDF file'),

        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': LaunchConfiguration('urdf_file')}],
        ),
        
        # RViz2 node to visualize the robot model
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
