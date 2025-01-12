cmake_minimum_required(VERSION 3.8)
project(module_2_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)




# ADD TASK 5 Executables here

add_executable(task_1 src/task_1.cpp)
ament_target_dependencies(task_1 rclcpp geometry_msgs)


add_executable(task_2 src/task_2.cpp)
ament_target_dependencies(task_2 rclcpp geometry_msgs)

add_executable(task_3 src/task_3.cpp)
ament_target_dependencies(task_3 rclcpp geometry_msgs)
# add_executable(task_3.1 src/task_3.1.cpp)


add_executable(led_publisher src/led_publisher.cpp)
ament_target_dependencies(led_publisher rclcpp std_msgs)

add_executable(led_subscriber src/led_subscriber.cpp)
ament_target_dependencies(led_subscriber rclcpp std_msgs)


install(TARGETS 
  task_1
  task_2
  task_3
  led_publisher
  led_subscriber
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})





ament_package()
