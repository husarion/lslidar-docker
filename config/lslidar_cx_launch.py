#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import LaunchConfigurationEquals

import os


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL", default_value=""),
        description="Robot model to use for the launch.",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    sensor_position_x = LaunchConfiguration("sensor_position_x")
    declare_sensor_position_x_arg = DeclareLaunchArgument(
        "sensor_position_x",
        default_value=EnvironmentVariable("SENSOR_POSITION_X", default_value="0.135"),
    )
    sensor_position_y = LaunchConfiguration("sensor_position_y")
    declare_sensor_position_y_arg = DeclareLaunchArgument(
        "sensor_position_y",
        default_value=EnvironmentVariable("SENSOR_POSITION_Y", default_value="0.0"),
    )
    sensor_position_z = LaunchConfiguration("sensor_position_z")
    declare_sensor_position_z_arg = DeclareLaunchArgument(
        "sensor_position_z",
        default_value=EnvironmentVariable("SENSOR_POSITION_Z", default_value="0.0"),
    )
    sensor_orientation_r = LaunchConfiguration("sensor_orientation_r")
    declare_sensor_orientation_r_arg = DeclareLaunchArgument(
        "sensor_orientation_r",
        default_value=EnvironmentVariable("SENSOR_ORIENTATION_R", default_value="-1.57"),
    )
    sensor_orientation_p = LaunchConfiguration("sensor_orientation_p")
    declare_sensor_orientation_p_arg = DeclareLaunchArgument(
        "sensor_orientation_p",
        default_value=EnvironmentVariable("SENSOR_ORIENTATION_P", default_value="0.0"),
    )
    sensor_orientation_y = LaunchConfiguration("sensor_orientation_y")
    declare_sensor_orientation_y_arg = DeclareLaunchArgument(
        "sensor_orientation_y",
        default_value=EnvironmentVariable("SENSOR_ORIENTATION_Y", default_value="0.0"),
    )

    mount_frame = LaunchConfiguration("mount_frame")
    declare_mount_frame_arg = DeclareLaunchArgument(
        "mount_frame",
        default_value=EnvironmentVariable("MOUNT_FRAME", default_value="cover_link"),
        description="Frame in which the sensor is mounted.",
    )

    publish_tf = LaunchConfiguration("publish_tf")
    declare_publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="true",
        description="Whether to publish the static transform between the mount frame and the laser frame.",
    )

    lsc16_driver_params = os.path.join(
        get_package_share_directory("lslidar_driver"), "params", "lslidar_cx.yaml"
    )

    sensor_namespace = "cx"
    if namespace != "":
        sensor_namespace = namespace + "/" + sensor_namespace

    driver_node = LifecycleNode(
        package="lslidar_driver",
        namespace=sensor_namespace,
        executable="lslidar_driver_node",
        name="lslidar_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[lsc16_driver_params],
    )

    if namespace != "":
        mount_frame =  namespace + "/" + mount_frame

    static_tf_node = Node(
        condition=LaunchConfigurationEquals("publish_tf", "true"),
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            sensor_position_x,
            sensor_position_y,
            sensor_position_z,
            sensor_orientation_r,
            sensor_orientation_p,
            sensor_orientation_y,
            mount_frame,
            "laser_link",
        ],
        output="screen",
    )

    actions = [driver_node, static_tf_node, declare_robot_model_arg,
               declare_namespace_arg, declare_sensor_position_x_arg,
               declare_sensor_position_y_arg, declare_sensor_position_z_arg,
               declare_sensor_orientation_r_arg, declare_sensor_orientation_p_arg,
               declare_sensor_orientation_y_arg, declare_mount_frame_arg,
               declare_publish_tf_arg]

    return LaunchDescription(actions)
