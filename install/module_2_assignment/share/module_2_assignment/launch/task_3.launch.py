from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Start Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
    )
    turtle_driver = Node(
        package='module_2_assignment',
        executable='task_3',
        name='turtle_driver',
         parameters=[
             {'cmd_vel_topic': '/turtle1/cmd_vel'}
         ]
    )
    

    # Spawn 5 turtles diagonally
    spawn_turtle1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 50.0, y: 1.0, theta: 0.0, name: 'turtle1'}\""],
        shell=True
    )
    spawn_turtle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 0.5, y: 10.5, theta: 0.0, name: 'turtle2'}\""],
        shell=True
    )
    spawn_turtle3 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 3, y: 8.0, theta: 0.0, name: 'turtle3'}\""],
        shell=True
    )
    spawn_turtle4 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 8.0, y: 3.0, theta: 0.0, name: 'turtle4'}\""],
        shell=True
    )
    spawn_turtle5 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
             "\"{x: 10.5, y: 0.5, theta: 0.0, name: 'turtle5'}\""],
        shell=True
    )
    

    # Drive the middle 3 turtles back and forth
    # drive_turtle2 = Node(
    #     package='module_2_assignment',
    #     executable='task_3',
    #     name='drive_turtle2',
    #     parameters=[
    #         {'cmd_vel_topic': '/turtle2/cmd_vel'}
    #     ]
    # )
    drive_turtle3 = Node(
        package='module_2_assignment',
        executable='task_3',
        name='drive_turtle3',
        parameters=[
            {'cmd_vel_topic': '/turtle3/cmd_vel'}
        ]
    )
    drive_turtle4 = Node(
        package='module_2_assignment',
        executable='task_3',
        name='drive_turtle4',
        parameters=[
            {'cmd_vel_topic': '/turtle4/cmd_vel'}
        ]
    )

    return LaunchDescription([
        turtlesim_node,
        turtle_driver,
        spawn_turtle1,
        spawn_turtle2,
        spawn_turtle3,
        spawn_turtle4,
        spawn_turtle5,
        #drive_turtle2,
        drive_turtle3,
        drive_turtle4,
    ])
