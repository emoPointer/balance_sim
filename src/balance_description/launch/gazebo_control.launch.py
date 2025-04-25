from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui_gazebo",
            default_value="true",
            description="Start gazebo automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui_rviz = LaunchConfiguration("gui_rviz")
    gui_gazebo = LaunchConfiguration("gui_gazebo")

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
    ),
    launch_arguments={
        "verbose": "false",
        "gui": gui_gazebo
    }.items(),
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("balance_description"), "urdf", "balance.xacro"]
            ),
            " ",
            # "use_gazebo_classic:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("balance_description"), "rviz", "balance_display.rviz"]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "balance",
            "-x", "0.0", "-y", "0.0", "-z", "0.5","-Y", "0"
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui_rviz),
    )

    nodes = [
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
