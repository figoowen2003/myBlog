---
title: ROS2工具之组件进阶操作
date: 2021-08-30 10:15:01
tags:
---

# 卸载组件

- 新建命令行窗口，启动一个组件容器

  ```
  ros2 run rclcpp_components component_container
  ```

  确认容器是否已经运行

  ```
  ros2 component list
  ```

  ![image-20210830111313540](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件进阶操作/image-20210830111313540.png)

- 新建第二个命令行窗口，加载组件talker和listener

  ```
  ros2 component load /ComponentManager composition composition::Talker
  ```

  ```
  ros2 component load /ComponentManager composition composition::Listener
  ```

  ![image-20210830111450260](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件进阶操作/image-20210830111450260.png)

- 使用unique ID去卸载组件节点

  ```
  ros2 component unload /ComponentManager 1 2
  ```

  ![image-20210830111822918](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件进阶操作/image-20210830111822918.png)



# 组件名和命名空间的重映射

- 新建命令行窗口，启动组件容器

  ```
  ros2 run rclcpp_components component_container
  ```

- 重映射组件名和命名空间

  ```
  # Remap node name
  ros2 component load /ComponentManager composition composition::Talker --node-name talker2
  # Remap namespace
  ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
  # Remap both
  ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
  ```

  ![image-20210830112549135](/home/ubuntu-ros2/myBlog/source/_posts/ROS2工具之组件进阶操作/image-20210830112549135.png)
