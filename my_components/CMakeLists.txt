cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include <my package>/include at compilation time
include_directories(include)

# Define preapproach Component
add_library(preapproach SHARED src/pre_approach.cpp)

# Add dependencies to preapproach Component 
ament_target_dependencies(preapproach
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs 
  sensor_msgs 
  tf2
  tf2_ros
)

# Register preapproach as Component so ROS 2 can use it
rclcpp_components_register_nodes(
  preapproach "my_components::PreApproach"
)

# Copy preapproach Component into those other folders
install(TARGETS 
  preapproach
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Copy <my package>/include into /include in the /install folder
install(DIRECTORY 
  include
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
