### Task 1: Create a Custom ROS 2 Node

- **Develop a ROS 2 node** that makes the Turtlesim follow a unique pattern:
  - **Circle Movement:** The turtle should move in a circle with a radius that is provided as a user input.
  - **Logarithmic Spiral Movement:** The turtle should move in a logarithmic spiral pattern.



**Node Functionality Explanation**
The node, MinimalPublisher, allows the user to select between two modes of movement:

**Circular Movement logic Explanation**

The turtle moves in a circle with a radius specified by the user. The user provides the radius as input during runtime. The angular velocity is calculated based on the radius:

Angular Velocity=Linear Velocity/Radius​
 
**Logarithmic Spiral MovementLogic Explanation** The turtle moves outward in a logarithmic spiral. The radius increases gradually during movement, causing the turtle to spiral outward. The linear velocity is proportional to the current radius, while the angular velocity remains constant.

1. Build the Node : "colcon build"
2. Build the workspace: 
         cd ~/assignment_ws
         colcon build --packages-select module_2_assignment
         source install/setup.bash
3. Run the Node
      Launch the Turtlesim node: '''ros2 run turtlesim turtlesim_node'''
4. Run the custom node: '''ros2 run module_2_assignment task_1'''
   
   Upon execution, a prompt message will appear in the terminal: 

   '''[INFO] [1736527635.772122036] [minimal_publisher]: Enter movement mode (1 for circle, 2 for spiral):'''

   cCase 1: Circular Movement

      If you select 1, another prompt will appear asking for the circle's radius:

      '''[INFO] [1736527765.870901739] [minimal_publisher]: Enter radius for the circle:'''

      After entering the desired radius (e.g., 2.5), the turtle will start moving in a circular path with the specified radius.
      
   Case 2: Spiral Movement
      If you select 2, the turtle will begin moving in a spiral pattern, where the radius gradually increases over time.








### Task 2: Develop a Launch File

- **Create a launch file** that starts the Turtlesim simulation and the custom ROS 2 node simultaneously.

- **Ensure proper documentation** of the node and launch file creation process, including the code and the results of executing the tasks.

1. Create the Launch File
2. Build the Package
   cd ~/assignment_ws
   colcon build --packages-select module_2_assignment
   source install/setup.bash
3. Execute the Launch File

   '''ros2 launch module_2_assignment task_2.launch.py'''
4. Expected Results
   On Running the Launch File

   Turtlesim Simulation:A Turtlesim window will open, showing the turtle in its default environment.

   Custom Node (turtle_driver):The turtle will start moving based on the default parameters:
   Mode: Circular movement (mode = 1).
   Radius: 1.0 (radius = 1.0).

   Logs will appear in the terminal, confirming the movement commands.

5. Results Documentation
      Logs Example
      When the launch file is executed, the terminal will display logs similar to the following:

      [INFO] [turtlesim]: Starting turtlesim_node...
      [INFO] [turtle_driver]: MinimalPublisher node initialized with mode: 1, radius: 1.0
      [INFO] [turtle_driver]: Circular Movement - Linear.x: 1.00, Angular.z: 1.00
      [INFO] [turtle_driver]: Circular Movement - Linear.x: 1.00, Angular.z: 1.00

6. Parameter Updates
      You can dynamically update parameters after the nodes are running:

      '''ros2 param set /turtle_driver mode 2'''
      '''ros2 param set /turtle_driver radius 2.5'''









### Task 3: Modify the Turtlesim Simulation Environment

- **Use existing Turtlesim services** such as `spawn` and `clear` to modify the simulation environment:
  - **Spawn 5 Turtlebots** with a single launch file, placing them diagonally from the top left to the bottom right.
  - **Drive the middle 3 turtles** back and forth continuously using ROS 2 services.

1. Execute the Launch File

   '''ros2 launch module_2_assignment task_3.launch.py'''






### Task 4: Modify Turtle Behavior with Parameters

- **Utilize ROS 2 parameters** to alter the behavior of the turtles:
  - **Change the speed** of the turtles dynamically during the simulation.

This task is completed in task 2. i.e parameter updates




### Task 5: Debugging a ROS 2 Node Using a Message Type
`Task Description:`

 Your task is to debug and fix a ROS 2 package that uses an uncommon message type, specifically std_msgs/msg/UInt8MultiArray. The node publishes and subscribes to this message type to simulate controlling the state of a robot's LEDs.

`Key Focus:`

- Correctly handle the UInt8MultiArray message type.
- Debug errors related to message publishing and subscribing.
- Fix issues in parameter handling.

task Completed

1. Execute the Launch File

   ''''ros2 launch module_2_assignment led_control.launch.py'''



