---
title: Navigation2专题四：设置里程计
date: 2021-11-03 16:14:29
tags: odometry, ekf
---

# 里程计简介

里程计系统是基于机器人的运动来对机器人的姿态和速度进行局部的精确估计。里程计信息的来源可以是IMU，LIDAR，RADAR，VIO以及车轮编码器等。需要注意的是，IMU会随时间发生漂移，而车轮编码器则是随着行进距离发生漂移，因此往往将它们一起使用来抵消彼此的负面特性。

- odom坐标系及其相关的变换使用机器人的里程计系统来发布定位信息，这个信息虽然连续但是会随着时间或距离（取决于传感器的模式和漂移）变得不太准确。即便如此，机器人仍然可以使用该信息来检测周边的人/物（如避障）

- map坐标系提供了全局的精确信息来纠正odom坐标系，以获得持续的精确的随时间变化的里程计信息。

- odom坐标系通过odom=>base_link的变换来连接到机器人系统的其他部分和Nav2。这个坐标变换既可以由tf2 broadcaster来发布，也可以由其他框架如robot_localization来发布。

- 除了需要odom=>base_link的变换外，Nav2还需要使用（其他节点）发布的nav_msgs/Odometry消息，因为这个消息中包含了机器人的速度信息，它的具体格式如下

  ```
  # This represents estimates of position and velocity in free space.
  # The pose in this message should be specified in the coordinate frame given by header.frame_id
  # The twist in this message should be specified in the coordinate frame given by the child_frame_id
  
  # Includes the frame id of the pose parent.
  std_msgs/Header header
  
  # Frame id the pose is pointing at. The twist is in this coordinate frame.
  string child_frame_id
  
  # Estimated pose that is typically relative to a fixed world frame.
  geometry_msgs/PoseWithCovariance pose
  
  # Estimated linear and angular velocity relative to child_frame_id.
  geometry_msgs/TwistWithCovariance twist
  ```

  这个消息提供了机器人位姿和速度的估计值，其中header消息提供了在给定的坐标系中带时间戳的数据，pose消息提供了机器人相对于指定坐标系（由header.frame_id指定）的位置和方向。twist消息给出了指定坐标系（child_frame_id指定）中的角速度和线速度。



# 设置机器人的里程计

机器人可以使用哪些里程计传感器决定了如何为它设置Nav的里程计系统。这里将使用一些基础的例子来帮助你配置机器人的Nav2。

以车轮编码器作为机器人的里程计来源。

- Nav2本身不需要车轮编码器。设置里程计的目的是计算里程计信息、发布nav_msgs/Odometry消息和odom=>base_link的转换。

- 车轮编码器信息到里程计信息的公式如下：

  ```
  linear = (right_wheel_est_vel + left_wheel_est_vel) / 2
  angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation;
  ```

  right_wheel_est_vel和left_wheel_est_vel分别是左右轮的估计速度，它们能通过获取车轮中轴随时间变化的位置关系来简单的获得。wheel_separation是车轮之间的距离。这些信息都被用于发布Nav2需要的消息，详情参考http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom/。

- 通过ros2_control框架来手动发布这些消息。ros2_control矿建包含了可以实时控制机器人的不同ROS2功能包。

  - 对于车轮编码器，提供了diff_drive_controller（位于ros2_controller功能包中），它订阅了cmd_vel话题，提取其中的geometry_msgs/Twist消息，计算出里程计信息并以odom话题为载体发布nav_msgs/Odometry消息。
  - 其他的功能包则会去处理其他类型的传感器。

- 始终记住，Nav2需要nav_msgs/Odometry和odom=>base_link变换，这就是设置里程计系统的目的。



# 使用Gazebo仿真一个Odometry系统

1. 前置条件，假设Gazebo已经安装

   - 安装gazebo_ros_pkgs来模拟里程计，并在Gazebo中用ROS2来控制机器人

     ```
     sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs
     ```

   - 修改sam_bot的URDF文件，原因是Gazebo使用SDF文件来描述一个机器人。幸运的是，Gazebo能够自动将特定格式的URDF文件转换为SDF文件。URDF文件能够兼容Gazebo的主要条件就是每一对<link>元素下都必须有<inertia>元素。在专题二中编写的URDF文件刚好满足了这个条件。

2. 在URDF文件中添加Gazebo控件

   将添加一个IMU传感器

   - GazeboRosImuSensor是一个传感器控件，它需要被添加到一个imu_link上。

   - imu_link将被<gazebo>元素所引用。

   - /demo/imu将被作为发布IMU信息的话题，同时根据REP145的约定，initalOrientationAsReference这个变量需要被设置为false

   - 此外还需要添加一些噪音到传感器配置中，依据是Gazebo的[sensor noise model](http://gazebosim.org/tutorials?tut=sensor_noise)

   - 具体操作为，将下面的内容添加到</robot>行的前面

     ```
     <link name="imu_link">
       <visual>
         <geometry>
           <box size="0.1 0.1 0.1"/>
         </geometry>
       </visual>
     
       <collision>
         <geometry>
           <box size="0.1 0.1 0.1"/>
         </geometry>
       </collision>
     
       <xacro:box_inertia m="0.1" w="0.1" d="0.1" h="0.1"/>
     </link>
     
     <joint name="imu_joint" type="fixed">
       <parent link="base_link"/>
       <child link="imu_link"/>
       <origin xyz="0 0 0.01"/>
     </joint>
     
      <gazebo reference="imu_link">
       <sensor name="imu_sensor" type="imu">
        <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
           <ros>
             <namespace>/demo</namespace>
             <remapping>~/out:=imu</remapping>
           </ros>
           <initial_orientation_as_reference>false</initial_orientation_as_reference>
         </plugin>
         <always_on>true</always_on>
         <update_rate>100</update_rate>
         <visualize>true</visualize>
         <imu>
           <angular_velocity>
             <x>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>2e-4</stddev>
                 <bias_mean>0.0000075</bias_mean>
                 <bias_stddev>0.0000008</bias_stddev>
               </noise>
             </x>
             <y>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>2e-4</stddev>
                 <bias_mean>0.0000075</bias_mean>
                 <bias_stddev>0.0000008</bias_stddev>
               </noise>
             </y>
             <z>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>2e-4</stddev>
                 <bias_mean>0.0000075</bias_mean>
                 <bias_stddev>0.0000008</bias_stddev>
               </noise>
             </z>
           </angular_velocity>
           <linear_acceleration>
             <x>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>1.7e-2</stddev>
                 <bias_mean>0.1</bias_mean>
                 <bias_stddev>0.001</bias_stddev>
               </noise>
             </x>
             <y>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>1.7e-2</stddev>
                 <bias_mean>0.1</bias_mean>
                 <bias_stddev>0.001</bias_stddev>
               </noise>
             </y>
             <z>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>1.7e-2</stddev>
                 <bias_mean>0.1</bias_mean>
                 <bias_stddev>0.001</bias_stddev>
               </noise>
             </z>
           </linear_acceleration>
         </imu>
       </sensor>
     </gazebo>
     ```

   添加一个差速驱动

   - 添加差速驱动模型插件（ModelPlugin），通过配置这个插件，便可以通过话题/demo/odom来发布nav_msgs/Odometry消息。

   - 左右轮的关节将被设置为sam_bot的车轮关节

   - 车轮分离距离和直径将根据wheel_ygap和wheel_radius的值来确定

   - 具体操作为，在标签</gazebo>下面一行插入以下内容

     ```
     <gazebo>
       <plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'>
         <ros>
           <namespace>/demo</namespace>
         </ros>
     
         <!-- wheels -->
         <left_joint>drivewhl_l_joint</left_joint>
         <right_joint>drivewhl_r_joint</right_joint>
     
         <!-- kinematics -->
         <wheel_separation>0.025</wheel_separation>
         <wheel_diameter>0.2</wheel_diameter>
     
         <!-- limits -->
         <max_wheel_torque>20</max_wheel_torque>
         <max_wheel_acceleration>1.0</max_wheel_acceleration>
     
         <!-- output -->
         <publish_odom>true</publish_odom>
         <publish_odom_tf>true</publish_odom_tf>
         <publish_wheel_tf>true</publish_wheel_tf>
     
         <odometry_frame>odom</odometry_frame>
         <robot_base_frame>base_link</robot_base_frame>
       </plugin>
     </gazebo>
     ```

3. 编译和运行

   - 编辑launch文件：删除joint state publiser的GUI节点，添加孵化sam_bot的节点

     删除

     ```
     joint_state_publisher_gui_node = launch_ros.actions.Node(
       package='joint_state_publisher_gui',
       executable='joint_state_publisher_gui',
       name='joint_state_publisher_gui',
       condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
     )
     ```

     ```
     joint_state_publisher_gui_node,
     ```

     添加

     ```
     launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
     ```

     ```
     spawn_entity = launch_ros.actions.Node(
       package='gazebo_ros',
       executable='spawn_entity.py',
       arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
       output='screen'
     )
     ```

     ```
           robot_state_publisher_node,
           spawn_entity,
           rviz_node
     ])
     ```

   - 修改package.xml

     删除

     ```
     <exec_depend>joint_state_publisher_gui</exec_depend>
     ```

   - 编译运行

     ```
     colcon build --packages-select sam_bot_description
     . install/setup.bash
     ros2 launch sam_bot_description display.launch.py
     ```

     ![image-20211103200025767](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211103200025767.png)
     
     查看是否发布了所需话题
     
     ```
     ros2 topic list
     ```
     
     ![image-20211104084714960](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211104084714960.png)
     
     ```
     ros2 topic /demo/imu
     ros2 topic /demo/odom
     ```
     
     ![image-20211104084841020](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211104084841020.png)
     
     可见/demo/imu话题发布了sensor_msgs/msg/Imu类型的消息，/demo/odom话题发布了nav_msgs/msg/Odometry类型的消息。这两个消息来自Gazebo对IMU传感器和差速驱动器的模拟。
     
     我们还需要注意，以上两个话题都没有订阅者。接下来我们将创建一个robot_localization节点去订阅它们，随后该节点还会使用两个话题发布的消息去为Nav2融合出一个局部精确且平滑的里程计信息。
     
     下图是rqt_graph的结果
     
     ![image-20211104090051306](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211104090051306.png)



# Robot Localization Demo

1. robot_localization功能包根据来自不同里程计传感器的数据，将他们融合成一个局部精确且平滑的历程及信息。它能够接收的数据类型包括nav_msgs/Odometry，sensor_msgs/Imu，geometry_msgs/PoseWithCovarianceStamped以及geometry_msgs/TwistWithCovarianceStamped。

2. robot_localization基本原理

   - 通常一个机器人至少以一个车轮编码器和一个IMU来组成它的里程计传感器。当多个传感器数据被提供给robot_localization后，它能够利用状态估计节点（state estimation nodes）来融合这些传感器数据。

   - 状态估计节点要么使用扩展的卡尔曼滤波（Extended Kalman filter, ekf_node），要么使用无迹卡尔曼滤波（Unscented Kalman filter, ukf_node）来实现融合。

   - 此外robot_localization功能包还实现了navsat_transform_node，它被用于在使用GPS时，将地理坐标系转换为机器人的时间坐标系。
   - 在使能robot_localization配置的条件下，融合的传感器数据将被以odometry/filtered以及accel/filtered两个话题的方式发布。
   - 此外robot_localization还将通过/tf话题去发布odom=>base_link的变换关系。
   - 如果机器人只有一个里程计源，我们依然可以使用robot_localization接收数据然后完成除了平滑之外的最小程度的处理。当然，在这种情况下，一个替代方式就是通过tf2的broadcaster在里程计节点的单一源中去发布变换。你也可以选择robot_localization去发布变换，并且也可以在输出结果中观测一些平滑的属性。

3. 配置Robot Localization

   - 本demo中我们将robot_localization配置为使用扩展卡尔曼滤波（ekf_node）来进行数据融合并发布odom=>base_link的变换。

   - 安装robot_localization功能包

     ```
     sudo apt install ros-<ros2-distro>-robot-localization
     ```

   - 使用一个YAML文件来指定ekf_node的参数。在工程根目录下创建config文件夹，在config目录中新建ekf.yaml文件，填入以下内容

     ```
     ### ekf config file ###
     ekf_filter_node:
         ros__parameters:
     # The frequency, in Hz, at which the filter will output a position estimate. Note that the filter will not begin
     # computation until it receives at least one message from one of theinputs. It will then run continuously at the
     # frequency specified here, regardless of whether it receives more measurements. Defaults to 30 if unspecified.
             frequency: 30.0
     
     # ekf_localization_node and ukf_localization_node both use a 3D omnidirectional motion model. If this parameter is
     # set to true, no 3D information will be used in your state estimate. Use this if you are operating in a planar
     # environment and want to ignore the effect of small variations in the ground plane that might otherwise be detected
     # by, for example, an IMU. Defaults to false if unspecified.
             two_d_mode: false
     
     # Whether to publish the acceleration state. Defaults to false if unspecified.
             publish_acceleration: true
     
     # Whether to broadcast the transformation over the /tf topic. Defaultsto true if unspecified.
             publish_tf: true
     
     # 1. Set the map_frame, odom_frame, and base_link frames to the appropriate frame names for your system.
     #     1a. If your system does not have a map_frame, just remove it, and make sure "world_frame" is set to the value of odom_frame.
     # 2. If you are fusing continuous position data such as wheel encoder odometry, visual odometry, or IMU data, set "world_frame"
     #    to your odom_frame value. This is the default behavior for robot_localization's state estimation nodes.
     # 3. If you are fusing global absolute position data that is subject to discrete jumps (e.g., GPS or position updates from landmark
     #    observations) then:
     #     3a. Set your "world_frame" to your map_frame value
     #     3b. MAKE SURE something else is generating the odom->base_link transform. Note that this can even be another state estimation node
     #         from robot_localization! However, that instance should *not* fuse the global data.
             map_frame: map              # Defaults to "map" if unspecified
             odom_frame: odom            # Defaults to "odom" if unspecified
             base_link_frame: base_link  # Defaults to "base_link" ifunspecified
             world_frame: odom           # Defaults to the value ofodom_frame if unspecified
     
             odom0: demo/odom
             odom0_config: [true,  true,  true,
                            false, false, false,
                            false, false, false,
                            false, false, true,
                            false, false, false]
     
             imu0: demo/imu
             imu0_config: [false, false, false,
                           true,  true,  true,
                           false, false, false,
                           false, false, false,
                           false, false, false]
     ```

     在上述配置文件yaml中，我们定义了frequency，two_d_mode，publish_acceleration，publish_tf，map_frame，odom_frame，base_link_frame以及world_frame这些参数的值。如果需要更详细的理解这些内容，可以参考[Parameters of state estimation nodes](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html#parameters)。

     为了添加一个传感器的输入到ekf_filter_node，需要在基本名字（odom，imu，pose，twist）之后添加数字序号。在本例中，只有一个nav_msgs/Odometry和一个sensor_msgs/Imu作为滤波器的输入，因此只需使用odom0和imu0。

     我们可以通过_config参数来指定一个传感器的哪些数值将被滤波器使用。这些数值的含义一次为x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpithc, vyaw, ax, ay, az。

     本demo中，我们将odom0_config的抵1，2，3和12个元素设置为true，这表示滤波器仅仅使用odom0的x, y, z和vyaw的值。

     在imu0_config矩阵中，只有roll，pitch和yaw被使用。典型的移动机器人级别的IMU还将提供角速度和线性加速度。

     为了确保robot_localization能够正常工作，不应该将那些可以彼此衍生的领域数据进行融合。比如角速度被融合到IMU内部以提供roll，pitch和yaw的估计值，我们就不应该在robot_localization中融合角速度。此外，我们不融合角速度还因为它在不使用所期望的高质量(昂贵)IMU时具有噪声特性。

4. 编译和运行

   - 将ekf_node添加到launch文件，在launch/display.lauch.py写入以下内容

     ```
     robot_localization_node = launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
     )
     ```

     然后需要在launch文件中声明use_sim_time参数，在return launch.LaunchDescription([模块中添加

     ```
     launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                                 description='Flag to enable use_sim_time'),
     ```

     最后在return语句中加入robot_localization

     ```
           robot_state_publisher_node,
           spawn_entity,
           robot_localization_node,
           rviz_node
     ])
     ```

   - 修改package.xml文件

     ```
     <exec_depend>robot_localization</exec_depend>
     ```

   - 修改CMakeList

     ```
     install(
       DIRECTORY src launch rviz config
       DESTINATION share/${PROJECT_NAME}
     )
     ```

   - 编译运行

     ```
     colcon build --packages-select sam_bot_description
     . install/setup.bash
     ros2 launch sam_bot_description display.launch.py
     ```

     ![image-20211111085910499](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211111085910499.png)

     ![image-20211104150107345](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211104150107345.png)

   - 查看话题和节点信息
   
     ```
     ros2 topic list
     ```

     ```
     ros2 topic info /demo/imu
     ros2 topic info /demo/odom
     ```
     
     ![image-20211110222653556](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211110222653556.png)

     **此处通过下载galactic版本的robot localization源码包，然后编译来解决ekf_node总是启动失败的问题：**

     ![image-20211110223024176](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211110223024176.png)

     ![image-20211110223108300](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211110223108300.png)
   
     **在sam_bot_description所在的工程目录下，source编译好的robot localization的setup.bash文件，然后再启动自己的launch文件**
     
     ![image-20211110223408257](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211110223408257.png)
     
     
   
     **此处有一个问题，应为当前使用的是rolling版本，结果robot_localization的ekf_node节点无法启动，因此，两个话题依然没有订阅者。**
     
     ![image-20211104183102352](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211104183102352.png)
     
     **探究器原因**
     
     ![image-20211110184228088](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211110184228088.png)
     
     ```
     ros2 run robot_localization ekf_node
     ```
     
     ![image-20211104183202653](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题四：设置里程计/image-20211104183202653.png)
     
     **应该是rolling版本的问题，在foxy版本则可以正常启动ekf_node**

