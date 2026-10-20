"""The detector with every debug and visualisation output on, and RViz to look at them.

The same detector as whycon.launch.py, run on a copy of the configuration with the pose, image,
TF, RViz marker and debug panel outputs switched on. RViz shows the detected markers next to the
ground truth, the annotated image and the three debug panels, laid out for a full-screen
3840x1600 window.

    ros2 launch whycode_vision whycon_debug.launch.py bag:=/root/data/whycode_1lap

With bag:= the bag is played on its own clock, in a loop. The ground truth needs whycode_sim
(its ground_truth_viz_node renders /marker_ground_truth); without it, RViz shows the detections
alone.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode

# Outputs switched on over the configuration's own values
DEBUG_OUTPUTS = (
    "publish_poses",
    "publish_images",
    "publish_tf",
    "publish_visualization_markers",
    "publish_debug_images",
)


def debug_config(path):
    """Writes a copy of the configuration with every output on, and returns its path."""
    with open(path) as source:
        config = yaml.safe_load(source)
    config["ros_interface"].update(dict.fromkeys(DEBUG_OUTPUTS, True))
    with tempfile.NamedTemporaryFile(
        "w", prefix="whycon_debug_", suffix=".yaml", delete=False
    ) as copy:
        yaml.safe_dump(config, copy, sort_keys=False)
    return copy.name


def launch_setup(context):
    share = get_package_share_directory("whycode_vision")
    config_file = debug_config(LaunchConfiguration("config_file").perform(context))
    bag = LaunchConfiguration("bag").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = bool(bag) if sim_time == "auto" else sim_time == "true"
    detector_parameters = [{"config_file": config_file}]

    actions = [
        SetParameter(name="use_sim_time", value=use_sim_time),
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: os.remove(config_file))),
    ]

    if LaunchConfiguration("use_composition").perform(context) == "true":
        actions.append(
            ComposableNodeContainer(
                name="whycon_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="whycode_vision",
                        plugin="whycon::WhyconComponent",
                        name="whycon",
                        parameters=detector_parameters,
                    )
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="whycode_vision",
                executable="whycon",
                name="whycon",
                output="screen",
                parameters=detector_parameters,
            )
        )

    if bag:
        play = ["ros2", "bag", "play", bag, "--clock", "--rate"]
        play.append(LaunchConfiguration("rate").perform(context))
        if LaunchConfiguration("loop").perform(context) == "true":
            play.append("--loop")
        actions.append(ExecuteProcess(cmd=play, output="screen"))

    if LaunchConfiguration("ground_truth").perform(context) == "true":
        try:
            get_package_share_directory("whycode_sim")
            actions.append(
                Node(
                    package="whycode_sim",
                    executable="ground_truth_viz_node",
                    name="marker_ground_truth_viz",
                    output="screen",
                    parameters=[
                        {
                            "marker_size_m": float(
                                LaunchConfiguration("marker_size_m").perform(context)
                            )
                        }
                    ],
                )
            )
        except PackageNotFoundError:
            pass

    if LaunchConfiguration("rviz").perform(context) == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(share, "config", "whycon_debug.rviz")],
            )
        )

    if LaunchConfiguration("image_view").perform(context) == "true":
        actions.append(
            Node(
                package="image_view",
                executable="image_view",
                name="image_view",
                output="screen",
                remappings=[("image", "/whycon/image_out")],
                parameters=[{"autosize": True}],
            )
        )
    return actions


def generate_launch_description():
    share = get_package_share_directory("whycode_vision")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(share, "config", "whycon_config_isaac.yaml"),
        description="Detector configuration; its outputs are all switched on",
    )

    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="false",
        description="Run WhyCon as a composable node in a component container",
    )

    bag_arg = DeclareLaunchArgument(
        "bag",
        default_value="",
        description="Bag to play on its own clock; empty to use the live image topic",
    )

    rate_arg = DeclareLaunchArgument("rate", default_value="1.0", description="Bag playback rate")

    loop_arg = DeclareLaunchArgument(
        "loop", default_value="true", description="Restart the bag when it ends"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="auto",
        description="true, false, or auto: true when a bag is played",
    )

    ground_truth_arg = DeclareLaunchArgument(
        "ground_truth",
        default_value="true",
        description="Render /marker_ground_truth in RViz (needs whycode_sim)",
    )

    marker_size_arg = DeclareLaunchArgument(
        "marker_size_m",
        default_value="0.2",
        description="Edge of the ground-truth plates, the printed marker's size",
    )

    rviz_arg = DeclareLaunchArgument("rviz", default_value="true", description="Open RViz")

    image_view_arg = DeclareLaunchArgument(
        "image_view",
        default_value="false",
        description="Launch image_view on /whycon/image_out",
    )

    return LaunchDescription(
        [
            config_file_arg,
            use_composition_arg,
            bag_arg,
            rate_arg,
            loop_arg,
            use_sim_time_arg,
            ground_truth_arg,
            marker_size_arg,
            rviz_arg,
            image_view_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
