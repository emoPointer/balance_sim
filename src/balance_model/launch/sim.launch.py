import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name='balance_model'

    balance = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'display.launch.py'    
                    )]), launch_arguments={'use_sim_time': 'true'}.items()
            )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
                )]),launch_arguments={'paused': 'false' }.items()
            )

    spawn_entity = Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description', 
                        '-entity', 'balance', 
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.5',
                        '-Y', '1.57'
                    ],
                    output='screen',
                )
    
    

    joint_controller_spawner = Node(
        package="controller_manager",
        executable='spawner',
        arguments=["joint_controller"],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable='spawner',
        arguments=["wheel_controller"],
    )

    return LaunchDescription([
        balance,
        gazebo,
        spawn_entity,
        joint_controller_spawner,
        wheel_controller_spawner,
    ])
