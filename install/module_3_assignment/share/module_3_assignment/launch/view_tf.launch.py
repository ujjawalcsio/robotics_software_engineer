import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the URDF file path
    urdf_file = os.path.join(
        '/home/her0/assignment_ws/src/robotics_software_engineer/module_3_assignment', 'urdf', 'three_dof_arm.urdf'
    )

    # Read the URDF file contents
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Print the URDF file path for debugging
    print(f"URDF File Path: {urdf_file}")

    return LaunchDescription([
        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
