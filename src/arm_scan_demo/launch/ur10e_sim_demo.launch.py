"""
Launch stock UR10e Gazebo + MoveIt, then start arm_scan_demo.
No custom tool description in this test.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        }.items(),
    )

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

    return LaunchDescription(
        [
            ur_gazebo_moveit,
            scan_demo_node,
        ]
    )