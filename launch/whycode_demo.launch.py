from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("whycode_vision")

    # Declare arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="whycode_2.world",
        description="World file to load (e.g. docking.world, whycode_1.world, whycode_2.world, whycode_3.world)",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo GUI (true/false)",
    )

    image_view_arg = DeclareLaunchArgument(
        "image_view",
        default_value="true",
        description="Launch image_view to visualize results (true/false)",
    )

    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="false",
        description="Use composable node container for WhyCon",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([package_share, "config", "whycon_config_sim.yaml"]),
        description="Path to WhyCon configuration file",
    )

    # Main Whycode Detection Launch
    whycon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("whycode_vision"), "/launch/whycon.launch.py"]
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
            "image_view": LaunchConfiguration("image_view"),
            "use_composition": LaunchConfiguration("use_composition"),
        }.items(),
    )

    # Simulation Launch
    whycode_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("whycode_sim"), "/launch/main.launch.py"]),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    return LaunchDescription(
        [
            world_arg,
            gui_arg,
            image_view_arg,
            use_composition_arg,
            config_file_arg,
            whycon_launch,
            whycode_sim_launch,
        ]
    )
