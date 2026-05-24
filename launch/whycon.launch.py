from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("whycon_whycode_localization")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([package_share, "config", "whycon_config_sim.yaml"]),
        description="Path to whycon configuration YAML",
    )

    image_view_arg = DeclareLaunchArgument(
        "image_view",
        default_value="false",
        description="Launch image_view on /whycon/image_out",
    )

    whycon_node = Node(
        package="whycon_whycode_localization",
        executable="whycon",
        name="whycon",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
    )

    image_view_node = Node(
        condition=IfCondition(LaunchConfiguration("image_view")),
        package="image_view",
        executable="image_view",
        name="image_view",
        output="screen",
        remappings=[("image", "/whycon/image_out")],
        parameters=[{"autosize": True}],
    )

    return LaunchDescription([
        config_file_arg,
        image_view_arg,
        whycon_node,
        image_view_node,
    ])
