cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# Create a custom message in ${PROJECT_NAME}
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/GoToLoading.srv"
)

# Create ${PROJECT_NAME}_node executable 
add_executable(${PROJECT_NAME}_node src/pre_approach.cpp)

# Add dependencies to ${PROJECT_NAME}_node executable
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  nav_msgs 
  sensor_msgs 
  tf2
  tf2_ros
)

# Create approach_service_node executable 
add_executable(approach_service_node src/approach_service_server.cpp)

# Add dependencies to approach_service_node executable
ament_target_dependencies(approach_service_node
  rclcpp
  geometry_msgs
  nav_msgs 
  sensor_msgs 
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create ${PROJECT_NAME}_v2_node executable 
add_executable(${PROJECT_NAME}_v2_node src/pre_approach_v2.cpp)

# Add dependencies to ${PROJECT_NAME}_v2_node executable
ament_target_dependencies(${PROJECT_NAME}_v2_node
  rclcpp
  geometry_msgs
  nav_msgs 
  sensor_msgs 
  tf2
  tf2_ros
)

# Link the C++ code generated to create the custom message(s)
# to approach_service_node 
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(approach_service_node
  "${cpp_typesupport_target}"
)
target_link_libraries(${PROJECT_NAME}_v2_node
  "${cpp_typesupport_target}"
)

# Copy ${PROJECT_NAME}_node executable into /install of its package
# so ROS2 can find it when doing ros2 run
install(TARGETS 
	${PROJECT_NAME}_node
	DESTINATION lib/${PROJECT_NAME})

# Copy approach_service_node executable into /install of its package
# so ROS2 can find it when doing ros2 run
install(TARGETS 
	approach_service_node
	DESTINATION lib/${PROJECT_NAME})

# Copy ${PROJECT_NAME}_v2_node executable into /install of its package
# so ROS2 can find it when doing ros2 run
install(TARGETS 
	${PROJECT_NAME}_v2_node
	DESTINATION lib/${PROJECT_NAME})

# Copy those folders in /share of the package
# so ROS2 can access all the non-executable files inside it
install(DIRECTORY
	launch
	rviz
	srv
	DESTINATION share/${PROJECT_NAME})

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
