import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="mk0_description"
    ).find("mk0_description")
    bringup_share = launch_ros.substitutions.FindPackageShare(
        package="mk0_bringup"
    ).find("mk0_bringup")
    default_model_path = os.path.join(
        pkg_share, "robots/mk0_hexagon_base.urdf.xacro"
    )
    default_world_path = os.path.join(pkg_share, "worlds/world_only.sdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")

    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description":
             ParameterValue(
                 Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
             },
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "mk0_robot", "-topic", "robot_description",
                   '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                   '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
        output="screen",
    )
    robot_localization_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(bringup_share, "config/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="world",
                default_value=default_world_path,
                description="Absolute path to world model file to load."
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    LaunchConfiguration("world")
                ],
                output="screen",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            spawn_entity,
            # robot_localization_node, # until I understand why this blows up the odom frame in simulation, this stays off.
        ]
    )
