from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() :

    package_name = "my_components"

    # Define Component Container
    my_container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
        
            # Component to load
            ComposableNode(
                package=package_name,
                plugin="my_components::PreApproach",
                name="preapproach_component"
            )
        ],
        output="screen"
    )

    # Path to RViz Configuration file
    rviz_config_file_path = PathJoinSubstitution([
                                FindPackageShare(package_name),
                                "rviz",
                                "config.rviz"
                            ])
    # Run RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file_path],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        rviz_node,
        my_container
    ])

