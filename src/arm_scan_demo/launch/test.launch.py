from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Custom UR10e + tool description.
    #
    # Passed as a full path, and description_package is deliberately left at
    # its ur_description default. UR's launch files use description_package for
    # more than locating this file: they also build
    # <description_package>/config/<ur_type>/{joint_limits,default_kinematics,
    # physical_parameters,visual_parameters}.yaml and feed those to xacro.
    # ur10e_tool_description ships only urdf/, so repointing description_package
    # at it breaks those paths. The full path survives because UR joins it onto
    # <description_package>/urdf/ and os.path.join drops the prefix when the
    # second component is absolute.
    custom_description = PathJoinSubstitution([
        FindPackageShare("ur10e_tool_description"),
        "urdf",
        "ur10e_with_tool.urdf.xacro",
    ])

    # Universal Robots Gazebo + MoveIt launch
    ur_sim_moveit_launch = PathJoinSubstitution([
        FindPackageShare("ur_simulation_gazebo"),
        "launch",
        "ur_sim_moveit.launch.py",
    ])

    ur_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_sim_moveit_launch),
        launch_arguments={
            "ur_type": "ur10e",
            "description_file": custom_description,
        }.items(),
    )

    # Start scan demo after Gazebo/MoveIt have had time to initialize
    scan_demo_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="arm_scan_demo",
                executable="move_group_interface",
                name="move_group_python_interface_tutorial",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        ur_gazebo_moveit,
        scan_demo_node,
    ])