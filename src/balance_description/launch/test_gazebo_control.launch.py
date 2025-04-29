import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,TimerAction,ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.parameter_descriptions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #获取功能包share路径
    urdf_package_path=get_package_share_directory('balance_description')
    
    default_urdf_path=os.path.join(urdf_package_path,'urdf','balance.xacro')

    #获取保存的gazebo世界路径
    # default_gazebo_world_path=os.path.join(urdf_package_path,'world','empty_world.world')

    #声明一个urdf参数，方便修改
    action_declare_arg_model_path=launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_urdf_path),description='加载的模型文件路径'
    )

    #通过文件路径获取内容。并转换参数值对象，以供传入robot_state_publisher
    #Command函数：创建一个执行命令的动作。通过cat命令得到urdf的内容。
    substitution_command_result=launch.substitutions.Command(
        ['xacro ',launch.substitutions.LaunchConfiguration('model')])
    #ParameterValue函数得到一个ros参数值对象
    robot_description_value=launch_ros.parameter_descriptions.ParameterValue(
        substitution_command_result,value_type=str)

    action_robot_state_publisher=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    #启动gazebo，并加载指定的世界文件。
    action_launch_gazebo=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
    ),
    launch_arguments={
        "verbose": "false",
        "gui": "true"
    }.items(),
    )

    #调用spawn_entity.py脚本，从ROS话题/robot_description读取机器人的URDF描述，并在Gazebo中生成名为jaw的实体。
    #-topic /robot_description：指定从该话题获取机器人的URDF模型数据（由robot_state_publisher节点发布），并将其转换成sdf格式。
    action_spawn_entity=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description',
                   '-entity','balance',
                   "-x", "0.0", "-y", "0.0", "-z", "0.3","-Y", "0.0"
                   ] 
    )

    # 此部分为ros2_control。实现启动时自动加载并激活 jaw_joint_state_broadcaster 控制器
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen'
    )

    # 加载并激活 jaw_effort_controller 控制器
    action_load_jaw_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','forward_position_controller'], 
        output='screen')

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        # 事件动作，当加载机器人结束后执行    
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],)
            ),
        # 事件动作，在关节状态发布后执行，发布力控制器
        launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=action_load_joint_state_controller,
            on_exit=[action_load_jaw_effort_controller],)
            ),
    ])