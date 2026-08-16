from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Custom UR10e + tool description
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