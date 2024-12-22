cmake_minimum_required(VERSION 3.8)
project(module_1_assignment)

# Enable compiler warnings for better debugging
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# Uncomment and add any dependencies here
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# Add the executable for task_1.cpp
add_executable(task_1 src/task_1.cpp)
add_executable(task_2 src/task_2.cpp)
add_executable(task_3 src/task_3.cpp)
add_executable(task_3.1 src/task_3.1.cpp)

# If there are dependencies, link them here
# For example:
# target_link_libraries(task_1 rclcpp)

# Install the executable
install(TARGETS task_1 task_2 task_3 task_3.1
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright linter
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint if in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
