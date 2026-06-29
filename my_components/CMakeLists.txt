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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include <my package>/include at compilation time
include_directories(include)

# Create a custom message in ${PROJECT_NAME}
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/GoToLoading.srv"
)

# Define preapproach_component Component
add_library(preapproach_component SHARED src/pre_approach.cpp)
# Define AttachServer Component
add_library(AttachServer SHARED src/attach_server.cpp)
# Define AttachClient Component
add_library(AttachClient SHARED src/attach_client.cpp)

# Define the macro COMPOSITION_BUILDING_DLL so that all elements 
# marked as COMPOSITION_PUBLIC be exported and usable from outsite 
# of the preapproach_component library
target_compile_definitions(preapproach_component PRIVATE COMPOSITION_BUILDING_DLL)

# Add dependencies to preapproach_component Component 
ament_target_dependencies(preapproach_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs 
  sensor_msgs 
  tf2
  tf2_ros
)
# Add dependencies to AttachServer Component 
ament_target_dependencies(AttachServer
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs 
  sensor_msgs 
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# Add dependencies to AttachClient Component 
ament_target_dependencies(AttachClient
  rclcpp
  rclcpp_components
)

# Register preapproach_component as Component so ROS 2 can use it
rclcpp_components_register_nodes(
  preapproach_component "my_components::PreApproach"
)
# Register AttachServer as Component so ROS 2 can use it
rclcpp_components_register_nodes(
  AttachServer "my_components::AttachServer"
)
# Register AttachClient as Component so ROS 2 can use it
rclcpp_components_register_nodes(
  AttachClient "my_components::AttachClient"
)

# Create the manual_composition executable
add_executable(manual_composition src/manual_composition.cpp)

# Link the C++ code generated to create the custom message(s)
# to manual_composition
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Add to the AttachServer component the code for the service message 
# (generated service typesupport) 
target_link_libraries(AttachServer
  "${cpp_typesupport_target}"
)
# Add to the AttachClient component the code for the service message 
# (generated service typesupport) 
target_link_libraries(AttachClient
  "${cpp_typesupport_target}"
)

# Add to the manual_composition executable the library that I have created 
target_link_libraries(manual_composition
  AttachServer
)

# Add ROS Packages needed for the manual_composition executable
ament_target_dependencies(manual_composition
  rclcpp
)

# Copy Components into those other folders
install(TARGETS 
  preapproach_component
  AttachServer
  AttachClient
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME})

# Copy <my package>/include into /include in the /install folder
install(DIRECTORY 
  include
  DESTINATION include)

install(DIRECTORY
  launch
  rviz
  srv
  DESTINATION share/${PROJECT_NAME}
)

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
