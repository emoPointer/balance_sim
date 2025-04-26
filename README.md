# balance_sim
simulation for balance robot

`ros2 launch balance_description view_robot.launch.py`

rviz查看机器人模型

`ros2 launch balance_description gazebo_control.launch.py`

启动gz仿真，使用gz的hardware_interface

`ros2 launch balance_bringup balance.launch.py`

启动自己的接口，需要将`balance_control.xacro`的硬件插件改为自己写的，并且注释下面`gazebo`的插件

## 问题

1. `rviz`和`gazebo`加载模型的路径不统一

   使用`<xacro:property name="mesh_path" value="file://$(find balance_description)/meshes"/>`

2. 启动自己的接口时，能加载进去，但是机器人模型散架
3. 使用`gazebo`硬件接口时模型在rviz中乱动，但是gazebo中不显示，过一会后模型叠加在原点![image-20250426224244388](/home/emopointer/.config/Typora/typora-user-images/image-20250426224244388.png)
