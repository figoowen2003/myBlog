---
title: ROS2工具之组件加载
date: 2021-08-28 16:00:38
tags: composition
---

# 关于组件

## 1. 编写组件

ROS2中的组件仅仅被编译成共享库中，没有main函数。组件同时是rclcpp：：Node的子类，并且它一样可以创建发布者、订阅者、服务器和客户端。

一旦创建了一个组件，它就必须在索引中注册才能被发现。

```
add_library(talker_component SHARED
   src/talker_component.cpp)
rclcpp_components_register_nodes(talker_component "composition::Talker")
# To register multiple components in the same shared library, use multiple calls
# rclcpp_components_register_nodes(talker_component "composition::Talker2")
```

## 2. 使用组件

常见的三种使用方法：

- 启动一个container进程，调用container提供的ROS服务load传入的功能包名和库名指定的组件，并在运行的过程中开始执行组件。
- 创建一个包含了多个node的客户端可执行程序，这些节点会被编译到可执行程序中。这种方式要求每个组件都拥有各自的头文件。
- 创建launch文件，使用ros2 launch创建一个container进程去加载多个组件。



# 在单个进程中组合多个节点

## 1. 前置条件

- 已经安装了ros2 rolling或者foxy版本

- 查看工作区中已经注册的可用组件

  ```
  ros2 component types composition
  ```

  ![image-20210828174916397](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828174916397.png)

## 2. 加载组件

- **使用ROS服务在运行时加载组件**

  新开一个终端，输入以下指令启动组件容器container

  ```
  ros2 run rclcpp_components component_container
  ```

  ![image-20210828175236424](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828175236424.png)

  新开第二个终端，查看当前所有的组件

  ```
  ros2 component list
  ```

  ![image-20210828175811970](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828175811970.png)

  查看/ComponentManager下的所有组件

  ```
  ros2 component list /ComponentManager
  ```

  ![image-20210828180150093](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828180150093.png)

  在第二个终端中输入以下命令，加载talker组件

  ```
  ros2 component load /ComponentManager composition composition::Talker
  ```

  ![image-20210828180414475](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828180414475.png)

  接着在第二个终端中输入以下命令，加载listener组件

  ```
  ros2 component load /ComponentManager composition composition::Listener
  ```

  ![image-20210828180926408](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828180926408.png)

  再次查看container中加载的容器

  ```
  ros2 component list
  ```

  ![image-20210828181104868](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828181104868.png)

  或者

  ![image-20210828181148045](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828181148045.png)

  运行结果

  ![image-20210828181603404](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828181603404.png)

  同理，可以在运行时加载一对server和client

  ```
  ros2 run rclcpp_components component_container
  ```

  ```
  ros2 component load /ComponentManager composition composition::Server
  ```

  ```
  ros2 component load /ComponentManager composition composition::Client
  ```

- 使用dlopen在运行时加载组件

  ```
  ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
  ```

  ![image-20210828182829017](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828182829017.png)

  代码实现如下

  ![image-20210828182952895](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828182952895.png)

- **使用ROS服务在编译时加载组件**

  执行composition功能包中的manual_composition节点

  ```
  ros2 run composition manual_composition
  ```

  ![image-20210828182121589](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828182121589.png)

  代码实现如下

  ![image-20210828182436169](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828182436169.png)

- 使用launch文件加载组件

  ```
  ros2 launch composition composition_demo.launch.py
  ```

  ![image-20210828183136493](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828183136493.png)

  ![image-20210828183248903](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件加载/image-20210828183248903.png)

