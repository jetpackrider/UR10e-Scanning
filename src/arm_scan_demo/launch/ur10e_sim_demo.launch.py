"""
Launch UR10e Gazebo + MoveIt with the custom tool description, then start
arm_scan_demo.

The scan node plans for the `tool_tip` link, which only exists in
ur10e_tool_description's xacro, so that description has to be handed to the UR
launch files here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_start_delay = LaunchConfiguration("scan_start_delay")

    declare_scan_start_delay = DeclareLaunchArgument(
        "scan_start_delay",
        default_value="15.0",
        description=(
            "Seconds to wait before starting the scan node. Must outlast "
            "Gazebo spawning the robot and the controller spawners coming up; "
            "raise it on slow machines or in a VM."
        ),
    )

    ur_sim_moveit_launch = PathJoinSubstitution(
        [
            FindPackageShare("ur_simulation_gazebo"),
            "launch",
            "ur_sim_moveit.launch.py",
        ]
    )

    ur_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_sim_moveit_launch),
        launch_arguments={
            "ur_type": "ur10e",
            # UR's launch files join description_file onto
            # <description_package>/urdf/, so it is a bare filename, not a path.
            "description_package": "ur10e_tool_description",
            "description_file": "ur10e_with_tool.urdf.xacro",
        }.items(),
    )

    scan_demo_node = TimerAction(
        period=scan_start_delay,
        actions=[
            Node(
                package="arm_scan_demo",
                executable="move_group_interface",
                name="move_group_python_interface_tutorial",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_scan_start_delay,
            ur_gazebo_moveit,
            scan_demo_node,
        ]
    )
