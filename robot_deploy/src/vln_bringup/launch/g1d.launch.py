"""Launch G1-D navigation with native odometry and a ZMQ camera by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context) -> list:
    odom_source = LaunchConfiguration("odom_source").perform(context)
    if odom_source not in {"unitree", "ros"}:
        raise ValueError("odom_source must be unitree or ros")
    camera_source = LaunchConfiguration("camera_source").perform(context)
    if camera_source not in {"ros", "zmq"}:
        raise ValueError("camera_source must be ros or zmq")
    endpoint = LaunchConfiguration("camera_endpoint").perform(context).strip()
    if camera_source == "zmq" and not endpoint:
        raise ValueError("camera_endpoint is required when camera_source=zmq")
    image_topic = LaunchConfiguration("image_topic").perform(context).strip()
    if not image_topic:
        image_topic = "camera/color/image_raw"
        if camera_source == "zmq":
            image_topic += "/compressed"
    image_parameters = {
        "image_topic": image_topic,
        "image_transport": (
            "compressed"
            if camera_source == "zmq"
            else LaunchConfiguration("image_transport").perform(context)
        ),
    }
    linear_limit = ParameterValue(LaunchConfiguration("max_linear_speed"), value_type=float)
    angular_limit = ParameterValue(LaunchConfiguration("max_angular_speed"), value_type=float)
    nodes = []
    if odom_source == "unitree":
        nodes.append(
            Node(
                package="g1d_adapter",
                executable="g1d_odometry",
                name="g1d_odometry",
                output="screen",
                parameters=[
                    {
                        "network_interface": LaunchConfiguration("network_interface"),
                        "dds_domain_id": ParameterValue(
                            LaunchConfiguration("dds_domain_id"), value_type=int
                        ),
                        "dds_topic": LaunchConfiguration("odom_dds_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "clock_offset_s": ParameterValue(
                            LaunchConfiguration("odom_clock_offset_s"), value_type=float
                        ),
                        "max_age_s": ParameterValue(
                            LaunchConfiguration("odom_max_age_s"), value_type=float
                        ),
                    }
                ],
            )
        )
    if camera_source == "zmq":
        nodes.append(
            Node(
                package="vln_camera",
                executable="zmq_camera",
                name="zmq_camera",
                output="screen",
                parameters=[
                    {
                        "endpoint": endpoint,
                        "camera_role": LaunchConfiguration("camera_role"),
                        "image_topic": image_topic,
                        "frame_id": LaunchConfiguration("camera_frame_id"),
                        "clock_offset_s": ParameterValue(
                            LaunchConfiguration("camera_clock_offset_s"), value_type=float
                        ),
                        "capture_delay_s": ParameterValue(
                            LaunchConfiguration("camera_capture_delay_s"), value_type=float
                        ),
                        "max_frame_age_s": ParameterValue(
                            LaunchConfiguration("camera_max_frame_age_s"), value_type=float
                        ),
                    }
                ],
            )
        )
    nodes.extend(
        [
            Node(
                package="vln_client",
                executable="vln_client",
                name="vln_client",
                output="screen",
                parameters=[
                    {
                        **image_parameters,
                        "server_url": LaunchConfiguration("server_url"),
                    }
                ],
            ),
            Node(
                package="vln_web",
                executable="vln_web",
                name="vln_web",
                output="screen",
                parameters=[
                    {
                        **image_parameters,
                        "manual_linear_limit": linear_limit,
                        "manual_angular_limit": angular_limit,
                        "manual_linear_accel": 0.5,
                        "manual_angular_accel": 0.5,
                    }
                ],
            ),
            Node(
                package="vln_mpc",
                executable="vln_mpc",
                name="vln_mpc",
                output="screen",
                parameters=[
                    {
                        "track_v_max": linear_limit,
                        "objnav_v_max": linear_limit,
                        "w_max": angular_limit,
                        "a_max_v": 0.5,
                        "a_max_w": 0.5,
                        "v_output_scale": 1.0,
                        "w_output_scale": 1.0,
                    }
                ],
            ),
            Node(
                package="g1d_adapter",
                executable="g1d_adapter",
                name="g1d_adapter",
                output="screen",
                parameters=[
                    {
                        "network_interface": LaunchConfiguration("network_interface"),
                        "dds_domain_id": ParameterValue(
                            LaunchConfiguration("dds_domain_id"), value_type=int
                        ),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "native_odom_dds_topic": (
                            LaunchConfiguration("odom_dds_topic") if odom_source == "unitree" else ""
                        ),
                        "max_linear_speed": linear_limit,
                        "max_angular_speed": angular_limit,
                    }
                ],
            ),
        ]
    )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "network_interface",
                description="Network interface connected to the G1-D DDS network",
            ),
            DeclareLaunchArgument("dds_domain_id", default_value="0"),
            DeclareLaunchArgument(
                "odom_source",
                default_value="unitree",
                choices=["unitree", "ros"],
                description="Bridge native Unitree DDS odometry or consume an existing ROS topic",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="g1d/odom",
                description="ROS odometry topic shared by the native bridge and motion adapter",
            ),
            DeclareLaunchArgument("odom_dds_topic", default_value="rt/agv/odom"),
            DeclareLaunchArgument("odom_clock_offset_s", default_value="0.0"),
            DeclareLaunchArgument("odom_max_age_s", default_value="0.5"),
            DeclareLaunchArgument(
                "camera_source",
                default_value="zmq",
                choices=["ros", "zmq"],
                description="Receive XRAW v2 JPEG over ZMQ by default, or use an existing ROS image topic",
            ),
            DeclareLaunchArgument(
                "camera_endpoint",
                default_value="tcp://192.168.100.78:5556",
                description="ZMQ publisher address; must be nonempty with camera_source=zmq",
            ),
            DeclareLaunchArgument("camera_role", default_value="head_fpv"),
            DeclareLaunchArgument("camera_frame_id", default_value="head_camera"),
            DeclareLaunchArgument("camera_clock_offset_s", default_value="0.0"),
            DeclareLaunchArgument("camera_capture_delay_s", default_value="0.0"),
            DeclareLaunchArgument("camera_max_frame_age_s", default_value="0.5"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="",
                description="Shared camera topic; default camera/color/image_raw, with /compressed for ZMQ",
            ),
            DeclareLaunchArgument(
                "image_transport",
                default_value="raw",
                choices=["raw", "compressed"],
                description="ROS input transport; forced to compressed when camera_source=zmq",
            ),
            DeclareLaunchArgument("server_url", default_value=""),
            DeclareLaunchArgument(
                "max_linear_speed",
                default_value="0.3",
                description="Adapter speed cap in m/s; must be positive and <= 1.5",
            ),
            DeclareLaunchArgument(
                "max_angular_speed",
                default_value="0.3",
                description="Adapter yaw-rate cap in rad/s; must be positive and <= 0.6",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
