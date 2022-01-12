---
title: Navigation专题二：TF2设置
date: 2021-10-29 16:49:59
tags: TF2, URDF, REP-105, REP-103
---

# 什么是坐标变换

## 坐标变换树(Transform tree)

- TF tree定义了不同坐标系之间的关系，包括坐标系间的平移、旋转和相对运动。由TF2 ROS功能包来发布这个变换树。

- 以一个简单的小车为例，它有一个移动底座，底座上安装了一个激光雷达

  - 小车上定义了两个坐标系：一个对应于底座的中心，我们将它命名为base_link，另一个对应于雷达的中心，我们将它命名为base_laser。

  - 假设1：有一组来自雷达的数据，表示距离雷达中心点的测量距离，即这些数据是以base_laser为参考坐标系；

    假设2：需要利用雷达数据帮助小车躲避环境中的障碍物。

  - 需要将接收到的雷达扫描数值从base_laser坐标系转换到base_link坐标系下。

    ![../../_images/simple_robot.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题二：TF2设置/simple_robot.png)

  - 如上图所示，雷达位于底座中心点上方10cm、靠右侧20cm处。

    如果我们将数据从base_link变换到base_laser中，那么需要将数据做平移(x: 0.1m, y: 0.0m, z: 0.2m)，相反，从base_laser到base_link中，则是做平移(x：-0.1m，y：0.0m，z：-0.20m)

  - 上述坐标变换关系如果由开发者自己来管理，那么随着坐标系数量的增加，这个工作会变得非常痛苦。幸运的是，我们可以使用TF2来管理坐标系之间的关系。

    首先，需要将base_link和base_laser添加到坐标变换树中；从概念上来说，变换树中的一个节点就对应着一个坐标系，一条边则对应于从当前节点到其子节点所使用的变换。
    TF2使用树型结构来保证两个坐标系之间只有一种遍历的方式，并假设所有的边都是从父节点指向子节点。

    ![../../_images/tf_robot.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题二：TF2设置/tf_robot.png)

  - 上图是一个简单的变换树例子，创建两个节点，一个节点用于base_link，另一个用于base_laser。设置base_link为父节点，base_laser为子节点，因此他们的坐标变换关系为(x: 0.1m, y: 0.0m, z: 0.2m)。

    当建立起了这棵变换树后，就能很容易的实现雷达数据从base_laser到base_link坐标系的转换（调用TF2的库）

    

# 静态坐标变换发布器 Static Transform Publiser 

下文是从命令行发布静态坐标变化的例子

```
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link base_laser
```

```
ros2 run tf2_ros tf2_echo base_link base_laser
```

结果如下

```
At time 0.0
- Translation: [0.100, 0.000, 0.200]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

对于一个真实的机器人系统，我们会创建URDF文件，在该文件中定义坐标变换以及更多关于机器人的信息，使用robot_state_publiser来取代static_transform_publiser。



# Navigation2中的坐标变换

- 两个重要的ROS REP（REP代表了ROS使用的移动平台的坐标框架的命名约定和语义意义）

  - REP 105：移动平台的坐标系
  - REP 103：测量标准单位和坐标协议

  为了更好地集成和复用软件组件，驱动程序、模型和库的开发者需要一个共享约定来协调框架。坐标框架的共享约定为开发者创建移动基座的驱动程序和模型提供了规范。同样，库和应用程序的开发者也可以更容易地使用与次约定兼容的各种移动基座上的软件。

- 以base_link，odom和map坐标系来讲解REP 105。base_link是固定在机器人上的坐标系，如机器人本体或者旋转中心。odom坐标系是一个相对于机器人起始位置的固定坐标系，主要被用于体现距离的局部一致性。移动平台在odom中的位姿可以`随时间漂移`，没有任何界限。这种漂移使得odom坐标系作为长时间全局参考时用处下降。然而，机器人在odom下的位姿保证是连续的，这意味着移动平台在odom下的`位姿始终是平滑`的，没有离散的变化。新坐标值通常是根据前一个时刻坐标得到的。map坐标系是固定的世界坐标系，用于体现距离的全局一致性，其z轴指向上方。移动平台相对于map坐标系的位姿不会随时间发生漂移，地图帧不是连续的，移动平台在该地图帧中的位置可以随时发生离散式的跳跃变化。

- REP 103则是讨论了一些测量单位的标准和其他相关约定，以便能将不同ROS功能包集成问题维持在一个最小的水平。基本内容大致就是坐标系必须使用右手螺旋规则，Z指向上方，X指向前方，单位应给是SI单位。

- Navigation2中需要发布以下关系

  1. `map` => `odom`

     由一个处理定位和建图的ROS功能包提供（如gmapping，cartography，AMCL）。这个变换在使用的过程中持续更新，因此我们不会为其在机器人的TF树中设置静态值。总之，所有的ROS官宣的SLAM和定位功能包都可以自动的提供自转换。

  2. `odom` => `base_link`

     有里程计系统来发布（如车轮的编码器）。通常是使用robot_localization功能包通过传感器（IMU，车轮编码器，VIO等）的数据融合来计算的。

  3. `base_link` => `base_laser` (sensor base frames)

     属于其他类型的静态变化，与之类似的还有base_link => wheels, wheels => IMU等。Nav2中也是使用坐标变换树将不同传感器的数据或其他机器人部件的坐标系联系起来。通常使用Robot State Publiser 和URDF文件来提供这些变换。

  

# 小结

通常不适用static_transform_publiser来发布转换，大多数的转换还是通过Robot State Publisher、URDF来提供。



