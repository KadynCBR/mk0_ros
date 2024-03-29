import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="mk0_description"
    ).find("mk0_description")
    default_model_path = os.path.join(
        pkg_share, "robots/mk0_hexagon_base.urdf.xacro"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="false",
                description="Flag to enable use_sim_time",
            ),
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            rviz_node,
        ]
    )
