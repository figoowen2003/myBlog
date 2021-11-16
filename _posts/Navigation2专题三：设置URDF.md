---
title: Navigation2专题三：设置URDF
date: 2021-11-02 09:52:37
tags: URDF, rviz, gazebo
---

# URDF与Robot State Publisher

- 使用Navigation2的要求之一是提供从base_link到不同传感器以及坐标系的转换。这棵转换树既可以是一个简单的树，它只包含一个从base_link到laser_link的转换，也可以是一棵由固定在不同位置的多个传感器组成的树，每个传感器都拥有自己的坐标系。Robot State Publiser提供了发布这些坐标变换的功能。
- Robot State Publisher是一个与TF2交互的的ROS2功能包，它发布了所有必需的坐标变换，这些变换都能够从机器人的几何形状和结构中推导出来。我们需要为Robot State Publisher提供正确的URDF文件，它将根据URDF的内容自动处理需要发布的转换信息。这个方法在复杂的坐标变换中非常有用，当然在简单的变换树中也推荐使用该方法。
- Universal Robot Descriptor File（URDF）是描述机器人模型的一个XML文件。此处，它将被用于构建与机器人几何形状相关的变换树，也会被用于定义可视化组件如材质和meshes，以便在RVIZ中显示机器人模型，此外，它还能定义机器人的物理属性，然后将这些属性应用于物理仿真器如Gazebo，以模拟机器人与环境的交互。
- URDF支持Xacro（XML宏），这些宏用于消除URDF中的重复模块。



# 环境搭建

- 假设用户已经搭建好了ROS2的开发环境

- 安装以下ROS2功能包，<ros2-distro>表示ros的版本，如foxy，rolling，galactic等

  ```
  sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
  sudo apt install ros-<ros2-distro>-xacro
  ```

  ![image-20211102150554836](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：设置URDF/image-20211102150554836.png)

  ![image-20211102150655730](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：设置URDF/image-20211102150655730.png)

- 创建工作目录，初始化一个ROS2功能包

  ```
  ros2 pkg create --build-type ament_cmake sam_bot_description
  ```

  ![image-20211102150911726](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：设置URDF/image-20211102150911726.png)



# 编写URDF

1. 在src/description目录下新建文件sam_bot_description.urdf，填充以下内容

   ```
   <?xml version="1.0"?>
   <robot name="sam_bot" xmlns:xacro="http://ros.org/wiki/xacro">
   
   
   
   </robot>
   ```

2. 用xacro的属性定义一些会被URDF复用的常量

   ```
     <!-- Define robot constants -->
     <xacro:property name="base_width" value="0.31"/>
     <xacro:property name="base_length" value="0.42"/>
     <xacro:property name="base_height" value="0.18"/>
   
     <xacro:property name="wheel_radius" value="0.10"/>
     <xacro:property name="wheel_width" value="0.04"/>
     <xacro:property name="wheel_ygap" value="0.025"/>
     <xacro:property name="wheel_zoff" value="0.05"/>
     <xacro:property name="wheel_xoff" value="0.12"/>
   
     <xacro:property name="caster_xoff" value="0.14"/>
   ```

   名为base_*的属性定义的是与机器人本体相关的变量

   wheel_radius和wheel_width定义了机器人两个轮子的形状

   wheel_ygap定义了在y轴方向轮子与机器人本体的间隙

   wheel_zoff与wheel_xoff是后轮沿z轴与x轴方向的位置

   caster_xoff是万向轮在x轴方向的位置

3. 定义base_link作为机器人的本体，我们将它设置为一个矩形。在URDF中一个link表示机器人的刚性组件。再完成这些link定义之后，robot state publisher将根据这些定义来确定每个link的坐标系并发布它们之间的变换。

   ```
     <!-- Robot Base -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="${base_length} ${base_width} ${base_height}"/>
         </geometry>
         <material name="Cyan">
           <color rgba="0 1.0 1.0 1.0"/>
         </material>
       </visual>
     </link>
   ```

4. 定义base_footprint link。这个link是一个虚拟的link，它没有尺寸或碰撞区域。定义它的主要目的是让不同的功能包能够确定机器人投影到地面的中心。比如，Navigation2使用这个link来确定避障算法中圆形足迹的中心。

   在完成link的定义后，还需要定义joint元素来描述坐标系之间的运动学和动力学属性。我们使用一个fixed类型的joint将base_footprint放置到合适的位置，此处是机器人本体的中心在地面的投影。

   ```
     <!-- Robot Footprint -->
     <link name="base_footprint"/>
   
     <joint name="base_joint" type="fixed">
       <parent link="base_link"/>
       <child link="base_footprint"/>
       <origin xyz="0.0 0.0 ${-(wheel_radius+wheel_zoff)}" rpy="0 0 0"/>
     </joint>
   ```

5. 添加两个驱动轮。

   为了避免代码重复，我们使用xacro中的宏macro来定义一个通用的车轮模块

   ```
     <!-- Wheels -->
     <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
       <link name="${prefix}_link">
         <visual>
           <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
           <geometry>
               <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
           </geometry>
           <material name="Gray">
             <color rgba="0.5 0.5 0.5 1.0"/>
           </material>
         </visual>
       </link>
   
       <joint name="${prefix}_joint" type="continuous">
         <parent link="base_link"/>
         <child link="${prefix}_link"/>
         <origin xyz="${x_reflect*wheel_xoff} ${y_reflect*(base_width/2+wheel_ygap)} ${-wheel_zoff}" rpy="0 0 0"/>
         <axis xyz="0 1 0"/>
       </joint>
     </xacro:macro>
   
     <xacro:wheel prefix="drivewhl_l" x_reflect="-1" y_reflect="1" />
     <xacro:wheel prefix="drivewhl_r" x_reflect="-1" y_reflect="-1" />
   ```

   此处prefix是link和joint的前缀，x_reflect和y_reflect则是车轮相对于x轴和y轴的翻转，接下来是定义轮子的视觉属性，最后是轮子旋转的关节定义。
   最后的两行是调用宏定义去实例化需要的车轮

6. 添加万向轮

   ```
     <!-- Caster Wheel -->
     <link name="front_caster">
       <visual>
         <geometry>
           <sphere radius="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
         </geometry>
         <material name="Cyan">
           <color rgba="0 1.0 1.0 1.0"/>
         </material>
       </visual>
     </link>
   
     <joint name="caster_joint" type="fixed">
       <parent link="base_link"/>
       <child link="front_caster"/>
       <origin xyz="${caster_xoff} 0.0 ${-(base_height/2)}" rpy="0 0 0"/>
     </joint>
   ```

这样我们就创建了一个简单的差分驱动机器人。



# 编译运行

- 修改package.xml，添加以下内容

  ```
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz</exec_depend>
  <exec_depend>xacro</exec_depend>
  ```

- 编写launch文件

  - 新建launch文件夹和display.launch.py文件。
  - launch文件具有以下功能：启动一个robot state publiser节点；自动启动RVIZ

  ```
  import launch
  from launch.substitutions import Command, LaunchConfiguration
  import launch_ros
  import os
  
  def generate_launch_description():
      pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
      default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
      default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
  
      robot_state_publisher_node = launch_ros.actions.Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
      )
      joint_state_publisher_node = launch_ros.actions.Node(
          package='joint_state_publisher',
          executable='joint_state_publisher',
          name='joint_state_publisher',
          condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
      )
      joint_state_publisher_gui_node = launch_ros.actions.Node(
          package='joint_state_publisher_gui',
          executable='joint_state_publisher_gui',
          name='joint_state_publisher_gui',
          condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
      )
      rviz_node = launch_ros.actions.Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          output='screen',
          arguments=['-d', LaunchConfiguration('rvizconfig')],
      )
  
      return launch.LaunchDescription([
          launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                              description='Flag to enable joint_state_publisher_gui'),
          launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                              description='Absolute path to robot urdf file'),
          launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                              description='Absolute path to rviz config file'),
          joint_state_publisher_node,
          joint_state_publisher_gui_node,
          robot_state_publisher_node,
          rviz_node
      ])
  ```

  - 使用官方提供的RVIZ配置文件：创建文件夹rviz和文件urdf_config.rviz，写入以下内容

    ```
    Panels:
      - Class: rviz_common/Displays
        Help Height: 78
        Name: Displays
        Property Tree Widget:
          Expanded:
            - /Global Options1
            - /Status1
            - /RobotModel1/Links1
            - /TF1
          Splitter Ratio: 0.5
        Tree Height: 557
    Visualization Manager:
      Class: ""
      Displays:
        - Alpha: 0.5
          Cell Size: 1
          Class: rviz_default_plugins/Grid
          Color: 160; 160; 164
          Enabled: true
          Name: Grid
        - Alpha: 0.6
          Class: rviz_default_plugins/RobotModel
          Description Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /robot_description
          Enabled: true
          Name: RobotModel
          Visual Enabled: true
        - Class: rviz_default_plugins/TF
          Enabled: true
          Name: TF
          Marker Scale: 0.3
          Show Arrows: true
          Show Axes: true
          Show Names: true
      Enabled: true
      Global Options:
        Background Color: 48; 48; 48
        Fixed Frame: base_link
        Frame Rate: 30
      Name: root
      Tools:
        - Class: rviz_default_plugins/Interact
          Hide Inactive Objects: true
        - Class: rviz_default_plugins/MoveCamera
        - Class: rviz_default_plugins/Select
        - Class: rviz_default_plugins/FocusCamera
        - Class: rviz_default_plugins/Measure
          Line color: 128; 128; 0
      Transformation:
        Current:
          Class: rviz_default_plugins/TF
      Value: true
      Views:
        Current:
          Class: rviz_default_plugins/Orbit
          Name: Current View
          Target Frame: <Fixed Frame>
          Value: Orbit (rviz)
        Saved: ~
    ```

- 修改CMakeList

  ```
  install(
    DIRECTORY src launch rviz
    DESTINATION share/${PROJECT_NAME}
  )
  ```

- 编译运行

  ```
  colcon build --packages-select sam_bot_description
  ```

  ```
  . install/setup.bash
  ```

  ```
  ros2 launch sam_bot_description display.launch.py
  ```

  ![image-20211102191716819](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：设置URDF/image-20211102191716819.png)
  
  右侧的窗口便是joint state publisher的GUI，joint state publisher用于发布非固定关节的状态，拖动水平条会让关节旋转。



# 添加物理属性

- 让我们的URDF文件包含一些机器人的运动学属性，这样物理仿真器如Gazebo就可以利用这些信息去模拟机器人在虚拟环境中的行为。

- 需要给机器人的本体增加惯性矩阵，我们也是用macro方式来封装这个模块

  ```
    <!-- Define intertial property macros  -->
    <xacro:macro name="box_inertia" params="m w h d">
      <inertial>
        <origin xyz="0 0 0" rpy="${pi/2} 0 ${pi/2}"/>
        <mass value="${m}"/>
        <inertia ixx="${(m/12) * (h*h + d*d)}" ixy="0.0" ixz="0.0" iyy="${(m/12) * (w*w + d*d)}" iyz="0.0" izz="${(m/12) * (w*w + h*h)}"/>
      </inertial>
    </xacro:macro>
  
    <xacro:macro name="cylinder_inertia" params="m r h">
      <inertial>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0" />
        <mass value="${m}"/>
        <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy = "0" ixz = "0" iyy="${(m/12) * (3*r*r + h*h)}" iyz = "0" izz="${(m/2) * (r*r)}"/>
      </inertial>
    </xacro:macro>
  
    <xacro:macro name="sphere_inertia" params="m r">
      <inertial>
        <mass value="${m}"/>
        <inertia ixx="${(2/5) * m * (r*r)}" ixy="0.0" ixz="0.0" iyy="${(2/5) * m * (r*r)}" iyz="0.0" izz="${(2/5) * m * (r*r)}"/>
      </inertial>
    </xacro:macro>
  ```

  - 在base_link中添加collision标签，并使用box_inertia宏

    ```
        <collision>
          <geometry>
            <box size="${base_length} ${base_width} ${base_height}"/>
          </geometry>
        </collision>
    
        <xacro:box_inertia m="15" w="${base_width}" d="${base_length}" h="${base_height}"/>
    ```

  - 对驱动轮做同样的操作

    ```
          <collision>
            <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
            <geometry>
              <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
            </geometry>
          </collision>
    
          <xacro:cylinder_inertia m="0.5" r="${wheel_radius}" h="${wheel_width}"/>
    ```

  - 对万向轮做以上操作

    ```
        <collision>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry>
            <sphere radius="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
          </geometry>
        </collision>
    
        <xacro:sphere_inertia m="0.5" r="${(wheel_radius+wheel_zoff-(base_height/2))}"/>
    ```

- 编译运行

  ![image-20211102223057055](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：设置URDF/image-20211102223057055.png)
