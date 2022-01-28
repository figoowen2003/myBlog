---
title: Navigation2专题九补充：设置生命周期与组合节点
date: 2022-01-27 11:33:56
tags:
---

# 生命周期

ROS2中引入了生命周期的概念，用来系统地管理参与机器人操作的不同节点的启动和关闭。生命周期节点（Lifecycle node）的使用确保了所有节点在开始执行任务之前都成功实例化，并且如果出现任何未响应的节点，Nav2会关闭所有节点。

生命周期节点包含状态机转换机制，用于使能ROS2服务器中的确定性行为。Lifecycle Manager是Nav2中处理生命周期节点转换的组件，它对生命周期节点的状态进行转换，同时对系统的状态提供更大的控制。

## 生命周期节点的主要状态：

- **Unconfigured**：是生命周期节点被实例化之后的初始状态。

  ​							Lifecycle Manager通过实现Configurating这个转换方法将一个节点从Unconfigured状态切换为Inactive状态。这个方法会设置所有的配置参数，还会准备其他所需要的设置，比如分配内存、设置静态发布与订阅的话题等等。

- **Inactive**：处于Inactive状态的节点被允许重新配置其参数，但是不能执行任何处理操作。

  ​				Lifecycle Manager通过实现Activating方法完成节点从Inactive到Active状态的转换。

- **Active**：是节点的主状态。一个处于Active状态的节点被允许执行任何处理操作。

- **Finalized**：是节点声明周期结束后的状态。如果一个非Finalized状态的节点崩溃了，Lifecycle Manager会关闭系统来阻止任何严重故障的发生。在关闭系统的过程中，LM会执行必要的清理操作包括Deactivating, Cleaning, ShuttingDown，然后将节点状态转换为Finalized。

## 如何自定义生命周期节点

你可能希望将自己的节点集成到Nav2的框架中，亦或者将新的生命周期节点添加到你自己的系统中。

下面是官方的一个例子，其中新增了一个名义上（概念上）的生命周期节点sensor_driver，以便通过Nav2的Lifecycle Manager来控制它，这样能确保在激活导航之前传感器的反馈数据是有效的。

我们可以通过将sensor_driver节点添加到launch文件中来达到这个目的，具体内容如下：

```
lifecycle_nodes = ['sensor_driver',
                   'controller_server',
                   'smoother_server',
                   'planner_server',
                   'recoveries_server',
                   'bt_navigator',
                   'waypoint_follower']

...

Node(
    package='nav2_sensor_driver',
    executable='sensor_driver',
    name='sensor_driver',
    output='screen',
    parameters=[configured_params],
    remappings=remappings),

Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    output='screen',
    parameters=[{'autostart': autostart},
                {'node_names': lifecycle_nodes}]),
```

node_names参数用于设置Lifecycle Manager将要处理的节点，它接收一个有序的节点列表，以便通过Lifecycle转换来启动这些节点。lifecycle_nodes是launch文件定义的一个有序的节点列表，Lifecycle Manager将按照列表中的顺序依次对这些节点启动状态转换（执行Configurating和Activating方法），而如果是进行shutdown转换，则是按照相反的顺序来处理这些节点。因此sensor_driver向先于其他导航服务器几点启动，这样就可以保证传感器数据在导航服务器被激活之前是可用的。

Lifecycle Manager中的另外两个参数：autostart被置为true，意味着你希望在启动时就将转换节点的转台设置为Active，false则表示你需要手动触发Lifecycle Manager来完成系统状态转换；bond_time则是设置了等待时间以决定在一个节点没有响应的情况下何时继续转换下一个节点。



# 组合

组合是ROS2中的另一个新概念，它通过将多个节点放到一个进程中来减少CPU和内存的开销。在Nav2中，组合这个概念被用于在单个进程中组合所有的Nav2节点，而不是单独启动他们。这个特性对于在嵌入式平台上部署整个系统非常有用。

接下来，我们将举例说明如何将新的Nav2服务器（route_server）添加到系统中，有手动组合与动态组合两种方式。

## 手动组合

Nav2代码仓中提供了名为composed_bringup.cpp的文件，位于nav2_bringup功能包下（事实上，我并没有在代码仓中发现这个文件，只在commit和issue记录中看到了它）。我们可以按照以下步骤添加route_server。

1. 添加以下内容到composed_bringup.cpp

   ```
   #include "nav2_route_server/route_server.hpp"
   ```

   创建共享指针route_node并将其名称添加到navigation_node_names容器中

   ```
   auto route_node = std::make_shared<nav2_route_server::RouteServer>();
   navigation_node_names.push_back(route_node->get_name());
   ```

   将route_node放入threads来创建一个单线程执行器route_node

   ```
   threads.push_back(create_spin_thread(route_node));
   ```

2. 在你的package.xml文件中天剑nav2_route_server

   ```
   <exec_depend>nav2_route_server</exec_depend>
   ```

3. 修改CMakeList

   ```
   find_package(nav2_route_server REQUIRED)
   set(dependencies nav2_route_server)
   ```

# 动态组合

在动态组合中，我们将利用launch文件将不同的服务器组合到一个进程中。这个进程酱油ComposableNodeContainer容器来创建，该容器中填充着不同的组合节点。这个容器可以像任何其他Nav2节点一样去启动和使用。

1. 在launch文件中添加一个新的ComposableNode()实例来指向你选择的组件容器

   ```
   container = ComposableNodeContainer(
       name='my_container',
       namespace='',
       package='rclcpp_components',
       executable='component_container',
       composable_node_descriptions=[
           ComposableNode(
               package='nav2_route_server',
               plugin='nav2_route_server::RouteServer',
               name='nav2_route_server'),
       ],
       output='screen',
   )
   ```

2. 将包含该服务器的功能包添加到package.xml文件中

   ```
   <exec_depend>nav2_route_server</exec_depend>
   ```

请参阅composition_demo.launch.py](https://github.com/ros2/demos/blob/master/composition/launch/composition_demo.launch.py)中的示例。

```

  
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='composition',
                    plugin='composition::Talker',
                    name='talker'),
                ComposableNode(
                    package='composition',
                    plugin='composition::Listener',
                    name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
```

![image-20220128170925045](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题九补充：设置生命周期与组合节点/image-20220128170925045.png)
