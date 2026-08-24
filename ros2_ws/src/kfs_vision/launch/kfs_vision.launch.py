from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("kfs_vision")

    default_params = PathJoinSubstitution(
        [package_share, "config", "kfs_vision.yaml"]
    )
    default_model = PathJoinSubstitution(
        [package_share, "models", "exp.onnx"]
    )
    default_plane_config = PathJoinSubstitution(
        [package_share, "config", "kfs_plane_fit.json"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="ROS namespace"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS parameter YAML file",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="ONNX model path",
            ),
            DeclareLaunchArgument(
                "plane_config_path",
                default_value=default_plane_config,
                description="Plane-fit input JSON path",
            ),
            DeclareLaunchArgument(
                "plane_config_output_path",
                default_value="",
                description="Optional absolute JSON output path for GUI sliders",
            ),
            DeclareLaunchArgument(
                "show_gui",
                default_value="false",
                description="Enable OpenCV diagnostic windows",
            ),
            DeclareLaunchArgument(
                "log_level", default_value="info", description="ROS log level"
            ),
            Node(
                package="kfs_vision",
                executable="kfs_vision_node",
                name="kfs_vision_node",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "plane_config_path": LaunchConfiguration(
                            "plane_config_path"
                        ),
                        "plane_config_output_path": LaunchConfiguration(
                            "plane_config_output_path"
                        ),
                        "show_gui": ParameterValue(
                            LaunchConfiguration("show_gui"), value_type=bool
                        ),
                    },
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )

