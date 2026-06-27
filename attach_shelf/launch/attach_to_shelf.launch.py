from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch arguments
    obstacle_arg = DeclareLaunchArgument(
        "obstacle",
        default_value="0.3"
    )

    degrees_arg = DeclareLaunchArgument(
        "degrees",
        default_value="-90"
    )

    final_approach_arg = DeclareLaunchArgument(
        "final_approach",
        default_value="true"
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config_file_name",
        default_value="config.rviz"
    )

    # Approach Service Server node
    approach_service_node = Node(
        package="attach_shelf",
        executable="approach_service_node",
        name="approach_service_node",
        output="screen",
        emulate_tty=True
    )

    # Attach shelf node
    attach_shelf_node = Node(
        package="attach_shelf",
        executable="attach_shelf_v2_node",
        name="attach_shelf_v2_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "obstacle": LaunchConfiguration("obstacle"),
            "degrees": LaunchConfiguration("degrees"),
            "final_approach": LaunchConfiguration("final_approach"),
        }],
    )

    # Run RViz
    rviz_config_file_path = PathJoinSubstitution([
                                FindPackageShare("attach_shelf"),
                                "rviz",
                                LaunchConfiguration("rviz_config_file_name"),
                            ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file_path],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        rviz_config_arg,
        approach_service_node,
        attach_shelf_node,
        rviz_node
    ])