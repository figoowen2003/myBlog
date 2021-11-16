---
title: Navigation2专题七：设置导航插件
date: 2021-11-11 18:40:23
tags:
---

# 规划器与控制器服务器（Planner and Controller Servers）

- Nav2中实现导航算法的三大插件服务器：规划器、控制器和恢复服务器，它们都是运行在ROS的action服务器。
- 本文中的规划器服务器与控制器服务器是导航的核心，它们可以实现一个或多个算法插件，每个插件都拥有为特定的动作或者机器人状态量身定制的配置。
- 当然除了这两个服务器，其他服务器如恢复器、平滑器等等由于都不是基于硬件或者环境，而是基于应用来提供一般性的服务建议。
- 规划器服务器负责实现计算机器人路径的算法。比如，一个插件被配置为计算两个相关位置的最短距离，另一个插件则是计算可以覆盖机器人所在环境的全部路径。
- 控制器服务器负责生成机器人在本地环境中完成任务所需要的适当的控制工作。这些任务包括但不限于：跟随一条有规划器服务器产生的路径，沿着这条路径躲避动态障碍物，甚至是去扩展坞充电。
- 总而言之，规划器与控制器服务器就是一张由一个或多个插件构成的图集，其中每个插件都会适用于对应的环境、场景或任务。



# 如何选择算法插件

## 规划器服务器（Planner Server）

- 规划器服务器的算法插件使用由不同传感器捕获的环境信息来查找机器人的路径。一部分算法通过搜索环境网格空间来运行，另一部分则是在考虑路径可行性的同时去扩展机器人的可能状态。

  - NavFn Planner使用Dijkstra或者A*方法的导航功能规划器

    不能保证在狭小空间中为非圆形的机器人规划可行路径，因为它使用机器人的圆形足迹（通过近似机器人的最大横截面半径）和每个代价地图网格单元的碰撞检测。此外，它不适合于ackermann和足型机器人，因为它们有转向约束。它最适用于可以向任何方向形式或者安全旋转的机器人，如圆形差速器和圆形全向机器人。

  - Smac 2D Planner使用4或8个连通的领域和更平滑的多分辨率查询来实现2D A*算法

    支持任意形状的ackermann和足型机器人。也可以用于需要小心地导航以避免在高速下翻到、大黄或者倾倒负载的高速机器人。

  - Theta Star Planner使用任一视线来创建非离散的定向的路径，以此实现Theta*算法

    它基于一个State Lattice planner。该插件扩展了机器人状态空间的同时，确保路径符合机器人的动力学约束。它提供了最小控制集，能以最小配置支持任何形状和尺寸的差速、全向和ackermann车辆。
    
    | Plugin Name            | Supported Robot Types                                        |
    | ---------------------- | ------------------------------------------------------------ |
    | NavFn Planner          | Circular Differential, Circular Omnidirectional              |
    | Smac Planner 2D        |                                                              |
    | Theta Star Planner     |                                                              |
    | Smac Hybrid-A* Planner | Non-circular or Circular Ackermann, Non-circular or Circular Legged |
    | Smac Lattice Planner   | Non-circular Differential, Non-circular Omnidirectional      |

- 配置示例

  ```
  planner_server:
    ros__parameters:
      planner_plugins: ['GridBased']
      GridBased:
        plugin: 'nav2_navfn_planner/NavfnPlanner'
  ```

  planner_plugins参数为规划器插件名称列表，如GridBased，对于每个定义在planner_plugin列表中的插件，都需要在随后的plugin参数中指定对应加载的插件类型。

## 控制器服务器

- 默认的控制器插件为DWB controller。它利用可配置插件实现了一个修正后的动态窗口逼近算法（DWA）来计算机器人的控制指令。

  - 利用Trajectory Generator plugin生成可能轨迹的集合。

  - 利用一个或多个Critic plugin评估之前的轨迹，每个plugin都将基于他们的配置方式给出不同的结果（分数）。

    这些评估结果的总和决定了一条轨迹的总分，得分最高的轨迹将决定输出的命令速度。

- 不同控制器的使用范围

  - DWB controller用于圆形或非圆形差速以及圆形或非圆形的全向机器人；如果给定了一个Trajectory Generation plugin，他能在考虑机器人最小曲率约束的条件下生成一个可能的轨迹集合，那么DWB也可以用于配置ackermann和足式机器人。

  - TEB controller是MPC时间最优的控制器。它使用Timed Elastic Band(TEB) approach算法，基于机器人执行指令的时间，与障碍物的距离以及机器人动力学约束的可行性来优化机器人的轨迹。它可用于差速、全向、ackermann和足式机器人。

  - RPP（Regulated Pure Pursuit controller）使用了一个纯追踪算法的变体并附加了规则启发式函数来管理碰撞和速度约束。这种变体是为了满足服务或工业机器人的需求而实施的，它适用于差速、ackermann和足式机器人。

    | Plugin Name    | Supported Robot Types                            | Task                       |
    | -------------- | ------------------------------------------------ | -------------------------- |
    | DWB controller | Differential, Omnidirectional                    | Dynamic obstacle avoidance |
    | TEB Controller | Differential, Omnidirectional, Ackermann, Legged |                            |
    | RPP controller | Differential, Ackermann, Legged                  | Exact path following       |

- 配置示例

  ```
  planner_server:
    ros__parameters:
      controller_plugins: ["FollowPath"]
      FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"
  ```

  规划器服务器的示例配置如上所示。该`planner_plugins`参数接受映射的规划器插件名称列表。对于`planner_plugins`(`GridBased`在我们的示例中) 中定义的每个插件命名空间，我们在`plugin`参数中指定要加载的插件类型。然后必须根据要使用的算法在此命名空间中指定其他配置。详情请参阅[配置指南](https://navigation.ros.org/configuration/index.html)。
