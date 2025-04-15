from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'balance_model'
    
    pkg_path = FindPackageShare(package=package_name)
    xacro_path = PathJoinSubstitution([pkg_path, 'urdf', 'balance_model.xacro'])

    # 将 XACRO 转换为 URDF
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_path]
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', 'True'],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                ])
            ]),
            launch_arguments={'world': 'empty'}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description_content
            }]
        ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[{'use_sim_time': True}]
        # ),

        # 删除可能存在的旧实体
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity',
                 '{name: "balance_model"}'],
            output='screen',
            # 忽略删除失败的错误
            on_exit=Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'balance_model',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.6',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        )

        
    ])