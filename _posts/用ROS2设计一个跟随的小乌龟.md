---
title: 用ROS2设计一个跟随的小乌龟
date: 2021-08-18 11:40:10
tags: ROS2, TF2, C++
---

# 要求

1. 孵化两只小乌龟，分别为turtle1和turtle2
2. 利用键盘控制小乌龟turtle1的运动
3. 当小乌龟turtle1运动时，小乌龟turtle2会跟随turtle1运动



# 实现

## 分析

1. 小乌龟turtle1和turtle2都需要发布其自身相对与世界坐标系world的坐标信息
2. 订阅1中的坐标信息，将坐标信息转换成turtle1相对于turtle2的坐标系信息
3. 计算出turtle2跟随turtle1所需要的速度信息
4. 发布turtle2的速度信息，控制turtle2的运动

## 流程

1. 新建功能包，添加依赖关系
2. 根据ROS2中小乌龟的功能包，利用launch文件孵化第一只小乌龟turtle1
3. 编写service client代码，孵化第二只小乌龟turtle2
4. 编写发布者代码，发布两只小乌龟相对于世界坐标系的坐标信息
5. 编写控制代码，需要实现两个功能，首先是订阅两只乌龟的坐标信息，然后生成turtle2相对于turtle1的的速度信息并发布
6. 命令行启动键盘控制功能包，注意需要修改发布的topic的名称，使其与turtle1的速度topic名称一致

## 准备工作

1. 了解如何创建第二只乌龟，且不受键盘控制

   创建乌龟需要使用service，话题是spawn

   ```
   ros2 run turtlesim turtlesim_node
   ```

   启动一个乌龟的节点，然后查看其使用的服务和话题

   ![image-20210818162411172](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/image-20210818162411172.png)

```
ros2 service list -t
```

![image-20210818162539391](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/image-20210818162539391.png)

名为/spawn的service话题就是用来生成小乌龟的

2. 分析如何获取两只乌龟的坐标

   ```
   ros2 topic list -t
   ```

   ![image-20210818163210267](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/image-20210818163210267.png)

​      乌龟坐标信息对应的话题/turtle1/pose，类型为turtlesim/msg/Pose

```
ros2 interface show turtlesim/msg
```

​		![image-20210818164043771](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/image-20210818164043771.png)



# 代码

## 创建功能包

```
ros2 pkg create --build-type ament_cmake cpp_a_follow_turtle_pkg --dependencies rclcpp tf2 tf2_ros tf2_geometry_msgs std_msgs geometry_msgs turtlesim
```

得到以下目录结构

![image-20210818171408962](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/image-20210818171408962.png)

包括CMakeLists.txt，incluede，package.xml，src

其中package.xml内容如下

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cpp_a_follow_turtle_pkg</name>
  <version>0.0.0</version>
  <description>a follow turtle</description>
  <maintainer email="figoowen2003@126.com">ubuntu-ros</maintainer>
  <license>Apache 2.0</license>

  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>turtlesim</depend>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

生成的CMakeList.txt内容如下

```
cmake_minimum_required(VERSION 3.5)
project(cpp_following_turtle_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## 生成两只乌龟

创建一个名为new_turtle_spawner.cpp的文件，拷贝以下代码到文件中

```
/**
 *  spawn a new turtle
 */

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

using namespace std;

int main(int argc, char **argv) {
    // init, start node, forward command line arguments to ROS
    rclcpp::init(argc, argv);

    // create spawn node
    auto node = rclcpp::Node::make_shared("turtle_spawner");
    RCLCPP_INFO(node->get_logger(), "create turtle spawner");

    // create spawn service client
    auto spawnerClient = node->create_client<turtlesim::srv::Spawn>("/spawn");

    // init a new turtle info
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = "turtle2";
    request->x = 1.5;
    request->y = 2.0;
    request->theta = 3.333;

    // make sure service has been connected
    while (!spawnerClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service");
            return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // call the spawn service
    auto result = spawnerClient->async_send_request(request);
    //wait for result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto r = result.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result got, %s created sucessfully!", r->name.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "fail to get result, turtle created failed!");
    }

    // spin function
    rclcpp::spin(node);

    // shutdown
    rclcpp::shutdown();

    return 0;
}
```

新建名为launch的文件夹，在文件夹下创建名为follow_turtle.launch.py的文件

```
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    turtle1 = Node(
        package = 'turtlesim',
        executable = 'turtlesim_node',
        name = 'turtle1',
        output = 'screen',
    )

    turtle2 = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle_spawner',
        name = 'turtle2',
        output = 'screen',
    )    

    return LaunchDescription([       
        turtle1,
        turtle2,
    ])

```

在CMakeList.txt中添加如下内容

```
add_executable(turtle_spawner src/new_turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim)

install(TARGETS turtle_spawner
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)
```

编译

```
colcon build --packages-select cpp_a_follow_turtle_pkg
```

运行

```
ros2 launch cpp_a_follow_turtle_pkg follow_turtle.launch.py
```

![image-20210818174721902](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/image-20210818174721902.png)

## 发布两只乌龟的坐标信息

新建文件turtles_pose_pub.cpp，拷贝以下代码到文件中

```
/**
 * subscribe the pose of the two turtles,then broadcast
 * their coordinates to the world base
 */

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <iostream>

using namespace std;
using std::placeholders::_1;

rclcpp::Node::SharedPtr node;
string turtleName;

void CallBack(const turtlesim::msg::Pose::SharedPtr msg) {
    // create broadcastor for TF
    static auto tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped tfStamped;

    // get turtle pose and change it to transformStamped info
    tfStamped.header.stamp = rclcpp::Time();
    tfStamped.header.frame_id = "world";   // father frame
    // tfStamped.child_frame_id = "turtle1";  // child frame
    tfStamped.child_frame_id = turtleName;
    // x,y,z info
    tfStamped.transform.translation.x = msg->x;
    tfStamped.transform.translation.y = msg->y;
    tfStamped.transform.translation.z = 0.0;
    // use euler angle to get quaternion
    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, msg->theta);
    tfStamped.transform.rotation.x = qtn.x();
    tfStamped.transform.rotation.y = qtn.y();
    tfStamped.transform.rotation.z = qtn.z();
    tfStamped.transform.rotation.w = qtn.w();

    // publish tf
    tfBr->sendTransform(tfStamped);
}

int main(int argc, char **argv) {
    // init node, deliver args to node
    rclcpp::init(argc, argv);

    // create node
    node = rclcpp::Node::make_shared("turtle_pub");
    cout << "argc = " << argc << endl;
    cout << "argv[0] = " << argv[0] << endl;
    cout << "argv[1] = " << argv[1] << endl;
    cout << "argv[2] = " << argv[2] << endl;
    cout << "argv[3] = " << argv[3] << endl;
    cout << "argv[4] = " << argv[4] << endl;
    // parse the args from main
    if (argc != 5) {
        RCLCPP_INFO(node->get_logger(), "args not corrected!");
        return 0;
    } else {
        turtleName = argv[1];
        RCLCPP_INFO(node->get_logger(), "got turtle name: %s", turtleName.c_str());
    }

    // create subscribe to turtle pose
    auto pose_sub = node->create_subscription<turtlesim::msg::Pose>(turtleName + "/pose", 100,
        std::bind(&CallBack, _1));

    // ros2 spin function
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
```

在launch文件中增加

```
    turtle_pub1 = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle_pub',
        name = 'tf_caster1',
        arguments = ['turtle1'],
        output = 'screen',
    )

    turtle_pub2 = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle_pub',
        name = 'tf_caster2',
        arguments = ['turtle2'],
        output = 'screen',
    )
    
    ......
    
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'turtle_name',
        #     default_value=turtle_name,
        #     description='input turtle name'),        
        turtle1,
        turtle2,
        turtle_pub1,
        turtle_pub2,
    ])    
```

在CMakeList.txt中添加如下内容

```
add_executable(turtle_pub src/turtles_pose_pub.cpp)
ament_target_dependencies(turtle_pub rclcpp turtlesim tf2_ros geometry_msgs)

install(TARGETS turtle_pub
  DESTINATION lib/${PROJECT_NAME})
```

编译运行，打开rviz查看

![image-20210818175501543](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/image-20210818175501543.png)

可以看到turtle1，turtle2和世界坐标系world的相对关系已经发布出来了

## 完成乌龟跟随的控制器

新建文件turtle2_controller.cpp，添加以下代码

```
/**
 * subsribe the broadcasting info from turtle1 and turtle2, search the nearest 
 * tf info, then change the pose of turtle1 relative to turtle2,
 * calculate the linear and angular speed 
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

using namespace std;

int main(int argc, char **argv) {
    // init ros node
    rclcpp::init(argc, argv);

    // create node
    auto node = rclcpp::Node::make_shared("turtle2_controller");

    // create tf subsciber object, traditional pointer, need delete
    // tf2_ros::Buffer *buffer;
    // buffer = new tf2_ros::Buffer(node->get_clock());
    // tf2_ros::TransformListener tfListener(*buffer);
    // use smart pointer, safe
    // shared_ptr<tf2_ros::Buffer> buffer(new tf2_ros::Buffer(node->get_clock()));
    shared_ptr<tf2_ros::Buffer> buffer = make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_ros::TransformListener tfListener(*buffer);

    // create speed publiser
    auto velPub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 1000);
    RCLCPP_INFO(node->get_logger(), "vel pub created!");

    // loop for speed pub
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        // get the coordinate of turtle1 related to turtle2
        try {
            geometry_msgs::msg::TransformStamped transStamped = buffer->lookupTransform("turtle2", "turtle1", tf2::TimePointZero);
            RCLCPP_INFO(node->get_logger(), "son1 related to son2: father %s, child %s offset(%.2f, %.2f, %.2f)",
                transStamped.header.frame_id.c_str(),
                transStamped.child_frame_id.c_str(),
                transStamped.transform.translation.x,
                transStamped.transform.translation.y,
                transStamped.transform.translation.z    
            );

            // compose twist info
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.5 * sqrt(pow(transStamped.transform.translation.x, 2) + pow(transStamped.transform.translation.y, 2));
            twist.angular.z = 4 * atan2(transStamped.transform.translation.y, transStamped.transform.translation.x);

            // pub the twist info timely
            velPub->publish(twist);
        } catch (tf2::TransformException &e) {
            RCLCPP_WARN(node->get_logger(), "warning %s", e.what());
        }

        rate.sleep();
    }

    // spin
    rclcpp::spin(node); // if put spin into while loop, will throw exception

    rclcpp::shutdown();

    // delete buffer;

    return 0;
}
```

完整版launch文件

```
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    turtle1 = Node(
        package = 'turtlesim',
        executable = 'turtlesim_node',
        name = 'turtle1',
        output = 'screen',
    )

    turtle2 = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle_spawner',
        name = 'turtle2',
        output = 'screen',
    )

    turtle_pub1 = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle_pub',
        name = 'tf_caster1',
        arguments = ['turtle1'],
        output = 'screen',
    )

    turtle_pub2 = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle_pub',
        name = 'tf_caster2',
        arguments = ['turtle2'],
        output = 'screen',
    )

    turtle2_controller = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle2_controller',
        name = 'turtle2_controller',
        output = 'screen',
    )      

    return LaunchDescription([     
        turtle1,
        turtle2,
        turtle_pub1,
        turtle_pub2,
        turtle2_controller,
    ])

```

完整版的CMakeList.txt如下

```
cmake_minimum_required(VERSION 3.5)
project(cpp_a_follow_turtle_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_spawner src/new_turtle_spawner.cpp)
add_executable(turtle_pub src/turtles_pose_pub.cpp)
add_executable(turtle2_controller src/turtle2_controller.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim)
ament_target_dependencies(turtle_pub rclcpp turtlesim tf2_ros geometry_msgs)
ament_target_dependencies(turtle2_controller rclcpp tf2_ros geometry_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(TARGETS turtle_spawner
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS turtle_pub
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS turtle2_controller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

编译运行

![image-20210818175501543](/home/ubuntu-ros2/myBlog/source/_posts/用ROS2设计一个跟随的小乌龟/Peek 2021-08-18 18-09.gif)

## 启动键盘控制

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/turtle1/cmd_vel
```

此处是调用ROS2自带的功能包，需要将原本的话题cmd_vel修改为/turtle1/cmd_vel
