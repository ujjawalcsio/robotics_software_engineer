from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the Turtlesim simulation node
    turtlesim_node = Node(
        package='turtlesim',              # ROS package name
        executable='turtlesim_node',      # Executable for the Turtlesim simulation
        name='turtlesim',                 # Name of the node
    )

    # Launch the custom node controlling the Turtlesim
    turtle_driver = Node(
        package='module_2_assignment',    # Your package name
        executable='task_2',              # Executable for your custom node (compiled C++ file)
        name='turtle_driver',             # Name of the node
        parameters=[
            {"mode": 1},                  # Default movement mode: 1 for circular movement
            {"radius": 1.0}               # Default radius for circular movement
        ],
    )

    # Return a launch description containing both nodes
    return LaunchDescription([
        turtlesim_node,
        turtle_driver,
    ])
