---
title: 设计Joint State Publisher
date: 2021-09-08 11:23:09
tags:
---

# Python版本

此处使用了ROS2 WIKI上的一个实现https://docs.ros.org/en/foxy/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher.html，urdf_tutorial/state_publisher.py

```
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion	#导入四元数模块
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped #导入坐标变换相关模块

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        # 创建joint state的发布者
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        # 创建坐标变换的广播器
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)	# 发布的周期

        # robot state
        tilt = 0.
        tinc = degree
        swivel = 0.
        angle = 0.
        height = 0.
        hinc = 0.005
        base_link2base_footprint = 0.
        left_wheel2base_link = 0.
        right_wheel2base_link = 0.
        front_wheel2base_link = 0.
        back_wheel2base_link = 0.


        # 定义需要发布的左边变换消息
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'axis'
        # 定义需要发布的joint state消息
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()	# 设置时间戳
                joint_state.name = ['base_link2base_footprint', 'left_wheel2base_link', 'right_wheel2base_link', 
                    'front_wheel2base_link', 'back_wheel2base_link'] # 设置关节名称
                joint_state.position = [base_link2base_footprint, left_wheel2base_link, right_wheel2base_link, 
                    front_wheel2base_link, back_wheel2base_link]

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle)*2
                odom_trans.transform.translation.y = sin(angle)*2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                tilt += tinc
                if tilt < -0.5 or tilt > 0.0:
                    tinc *= -1
                height += hinc
                if height > 0.2 or height < 0.0:
                    hinc *= -1
                swivel += degree
                angle += degree/4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

# 欧拉角转四元数
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()

```



## C++版本

以下代码截取子ROS2官方的dummy robot，实现了一个节点作为joint state的发布器

```
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
	// 创建名为dummy_joint_states的节点
    auto node = rclcpp::Node::make_shared("dummy_joint_states");
	
	// 创建名为joint_states的发布者，发布的话题类型为sensor_msgs::msg::JointState
    auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

	// 设置发布周期为50ms
    rclcpp::WallRate loop_rate(50);

	// 设置joint state变量，name中存储机器人关节的名称，position存储每个关节对应的初始位置值
    sensor_msgs::msg::JointState msg;
    msg.name.push_back("single_rrbot_joint1");
    msg.name.push_back("single_rrbot_joint2");
    msg.position.push_back(0.0);
    msg.position.push_back(0.0);

	// TimeSource用于管理当前节点的时钟
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    auto counter = 0.0;
    auto joint_value = 0.0;
    while (rclcpp::ok()) {
      counter += 0.1;
      joint_value = std::sin(counter);

      for (size_t i = 0; i < msg.name.size(); ++i) {
        msg.position[i] = joint_value;
      }
	  
	  // 为消息添加时间戳
      msg.header.stamp = clock->now();

      joint_state_pub->publish(msg);
      rclcpp::spin_some(node);	// 在单一线程中执运行该节点，但是采用非阻塞形式，等同于ROS1中的spin_once
      loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

```

