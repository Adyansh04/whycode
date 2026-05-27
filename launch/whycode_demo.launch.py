from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("whycode_vision")

    # Main Whycode Detection Launch
    whycon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("whycode_vision"), "/launch/whycon.launch.py"]
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [package_share, "config", "whycon_config_sim.yaml"]
            ),
            "image_view": "true",
            "use_composition": "false",
        }.items(),
    )

    # Simulation Launch
    whycode_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("whycode_sim"), "/launch/main.launch.py"]),
        launch_arguments={
            # "world": "docking.world",
            # "world": "whycode.world",
            # "world": "whycode_1.world",
            "world": "whycode_2.world",
            # "world": "whycode_3.world",
            "gui": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            whycon_launch,
            whycode_sim_launch,
        ]
    )
