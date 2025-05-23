from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,TimerAction,ExecuteProcess
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
            default_value="true",
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

    node_joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{
            "use_gui": True,
            "rate": 30.0,
        }]
)

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "balance",
            "-x", "0.0", "-y", "0.0", "-z", "0.3","-Y", "0.0"
        ],
        output="screen",
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
        node_joint_state_publisher,
        spawn_entity,
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes)