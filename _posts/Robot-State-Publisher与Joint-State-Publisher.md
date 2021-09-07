---
title: Robot State Publisher与Joint State Publisher
date: 2021-09-07 16:30:23
tags:
---

# 机器人模型的Links和Joints

为了更好的理解Robot State Publisher和Joint State Publisher两个功能包的差异，我们这里先科普一下机器人的两大组成部分：Joints与Links。

Links（连杆）是机器人的刚性部件，可以理解为“骨头”；Joints（关节）是机器人的移动部分；Links通过Joints互相连接。

以人的手臂为例，肩、肘和腕是关节；大臂、小臂和手掌是连杆。

![link_joint](/home/ubuntu-ros2/myBlog/source/_posts/Robot-State-Publisher与Joint-State-Publisher/link_joint.jpg)

如果是机械臂，连杆和关节看起来就是这样

![链接关节机械臂](/home/ubuntu-ros2/myBlog/source/_posts/Robot-State-Publisher与Joint-State-Publisher/link-joint-robotic-arm.jpg)

机械臂有刚性部件（连杆）和非刚性部件（关节）组成 ,关节处的私服电机驱动机械臂连杆的移动。

对于带雷达的无人小车，连杆和关节如图所示：

![移动机器人关节链接](/home/ubuntu-ros2/myBlog/source/_posts/Robot-State-Publisher与Joint-State-Publisher/mobile-robot-joints-links.jpg)

车轮为revolution类型的关节，产生旋转于东。车轮关节将车轮刚性部分与本体连杆连接起来。

fixed类型的关节为固定关节，不产生移动。如LIDAR连杆就是通过一个fixed关节固定到机器人本体上。

prismatic类型关节常用于SCARA机器人，它将产生一个连杆间的线性移动。



# Joint State Publisher

每当我们希望机器人去完成指定任务时，必须有一种方法可以随时了解每个关节的位置和速度。这就是Joint State Publisher的作用。

Joint State Publisher会追踪机器人的位置信息（如伺服电机得到的弧度为单位的角度、线性发生器提产生的以米为单位的位移）以及每个关节的速度，然后将位置、速度的值以sensor_msgs/JointState的形式发布给ROS系统。



# Robot State Publisher

该发布器主要接收以下两种来源的输入：

- Joint State Publisher发布的sensor_msgs/JointState
- 一个URDF文件格式的机器人模型

Robot State Publisher的输出为机器人每个坐标系的位置和姿态，它将这些数据发布给tf2功能包。

tf2功能包负责持续的监控机器人全部坐标系的位置和姿态。在任意指定时间，都能通过查询tf2功能包获取任意坐标系相对其他坐标系（如child frame相对与parent frame）的位置和姿态。

比如，我们使用ROS2查询LIDAR连杆相对于机器人底座的坐标信息

```
ros2 run tf2_ros tf2_echo base_link lidar_link
```

语法形式

```
ros2 run tf2_ros tf2_echo <parent frame> <child frame>
```



# Joint State Publisher的实现

## 仿真环境下Gazebo的实现

在Gazebo环境中，需要使用joint state publisher Gazebo plugin去发布joints的位置和姿态，详情参考

http://docs.ros.org/en/jade/api/gazebo_plugins/html/gazebo__ros__joint__state__publisher_8h_source.html

```

00001 /*
00002  * Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
00003  *  All rights reserved.
00004  *
00005  *  Redistribution and use in source and binary forms, with or without
00006  *  modification, are permitted provided that the following conditions are met:
00007  *      * Redistributions of source code must retain the above copyright
00008  *      notice, this list of conditions and the following disclaimer.
00009  *      * Redistributions in binary form must reproduce the above copyright
00010  *      notice, this list of conditions and the following disclaimer in the
00011  *      documentation and/or other materials provided with the distribution.
00012  *      * Neither the name of the <organization> nor the
00013  *      names of its contributors may be used to endorse or promote products
00014  *      derived from this software without specific prior written permission.
00015  *
00016  *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
00017  *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
00018  *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
00019  *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
00020  *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
00021  *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
00022  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
00023  *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
00024  *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
00025  *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
00026  *
00027  **/
00028 
00029 
00030 #ifndef JOINT_STATE_PUBLISHER_PLUGIN_HH
00031 #define JOINT_STATE_PUBLISHER_PLUGIN_HH
00032 
00033 #include <boost/bind.hpp>
00034 #include <gazebo/gazebo.hh>
00035 #include <gazebo/physics/physics.hh>
00036 #include <gazebo/common/common.hh>
00037 #include <stdio.h>
00038 
00039 // ROS
00040 #include <ros/ros.h>
00041 #include <tf/transform_broadcaster.h>
00042 #include <sensor_msgs/JointState.h>
00043 
00044 // Usage in URDF:
00045 //   <gazebo>
00046 //       <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
00047 //              <robotNamespace>/pioneer2dx</robotNamespace>
00048 //              <jointName>chassis_swivel_joint, swivel_wheel_joint, left_hub_joint, right_hub_joint</jointName>
00049 //              <updateRate>100.0</updateRate>
00050 //              <alwaysOn>true</alwaysOn>
00051 //       </plugin>
00052 //   </gazebo>
00053       
00054 
00055 
00056 namespace gazebo {
00057 class GazeboRosJointStatePublisher : public ModelPlugin {
00058 public:
00059     GazeboRosJointStatePublisher();
00060     ~GazeboRosJointStatePublisher();
00061     void Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
00062     void OnUpdate ( const common::UpdateInfo & _info );
00063     void publishJointStates();
00064     // Pointer to the model
00065 private:
00066     event::ConnectionPtr updateConnection;
00067     physics::WorldPtr world_;
00068     physics::ModelPtr parent_;
00069     std::vector<physics::JointPtr> joints_;
00070 
00071     // ROS STUFF
00072     boost::shared_ptr<ros::NodeHandle> rosnode_;
00073     sensor_msgs::JointState joint_state_;
00074     ros::Publisher joint_state_publisher_;
00075     std::string tf_prefix_;
00076     std::string robot_namespace_;
00077     std::vector<std::string> joint_names_;
00078 
00079     // Update Rate
00080     double update_rate_;
00081     double update_period_;
00082     common::Time last_update_time_;
00083     
00084 };
00085 
00086 // Register this plugin with the simulator
00087 GZ_REGISTER_MODEL_PLUGIN ( GazeboRosJointStatePublisher )
00088 }
00089 
00090 #endif //JOINT_STATE_PUBLISHER_PLUGIN_HH
00091 
```

## 真实世界中的Joint State Publisher

需要自己去实现这个发布器，我们将在下一篇blog中介绍这个方法。
