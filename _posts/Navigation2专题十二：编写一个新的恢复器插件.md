---
title: Navigation2专题十二：编写一个新的恢复器插件
date: 2021-12-13 10:53:39
tags:
---

# 概述

恢复器插件（Recovery Plugin）位于恢复器服务器中，不同于规划器与控制器，每个恢复器都会持有自己独特的动作服务器。规划器和控制器在完成相同的任务时会使用相同的API。而由于恢复器可以执行多种任务，故每个恢复器都可以有自己独特的action消息定义及对应的动作服务器。这为恢复服务器的运行提供了巨大的灵活性。



# 要求

要求在本地机器上已经安装或构建好了以下软件包：

● ROS 2（[二进制](https://www.zhihu.com/search?q=二进制&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"article"%2C"sourceId"%3A389470354})安装或从源代码构建）

● Nav2（包括依赖包）

● Gazebo

● Turtlebot3



# 步骤

## 1. 创建插件

本教程中将实现一个简单的求助的恢复行为。它将使用Twilio通过SMS向远程操作中心发送消息。代码位于navigation_tutorials代码仓的nav2_sms_recovery目录下https://link.zhihu.com/?target=https%3A//github.com/ros-planning/navigation2_tutorials。

本例的插件实现了插件类nav2_core::Recovery。在nav2_recoveries中有一个很好的动作封装器nav2_recoveries::Recovory，它继承自nav2_core类，因而它能被用作一个插件，同时它又可以处理绝大多数的ROS2 action服务器的行为，我们把它作为教程中的基类。

基类提供了4个纯虚函数来实现恢复器插件。恢复器服务器将使用并管理该插件，但是每个插件都需要提供各自独一无二的动作服务器接口。

以下表格是不使用nav2_recoveries包装器的情形

| **Virtual method** | **Method description**                                       | **Requires override?** |
| ------------------ | ------------------------------------------------------------ | ---------------------- |
| configure()        | 当服务器进入on_configure状态时会调用此方法。理想情况下，此方法应该执行ROS参数声明和恢复器成员变量的初始化。此方法需要4个输入参数：指向父节点的共享指针、恢复器名称、tf缓冲区指针和指向碰撞检查器的共享指针。 | Yes                    |
| activate()         | 当恢复器服务器进入on_activate状态时会调用此方法。理想情况下，此方法应实现恢复器进入活动状态前的必要操作。 | Yes                    |
| deactivate()       | 当恢复器服务器进入on_deactivate状态时调用此方法。理想情况下，此方法应实现恢复器进入非活动状态前的必要操作。 | Yes                    |
| cleanup()          | 当恢复器服务器进入on_cleanup状态时会调用此方法。理想情况下，此方法应清除为恢复器创建的各种资源。 | Yes                    |

接下来是使用nav2_recoveries包装器的情形，它支持ROS2的action接口和模板操作

| **Virtual method** | **Method description**                                       | **Requires override?** |
| ------------------ | ------------------------------------------------------------ | ---------------------- |
| onRun()            | 当收到新的恢复动作请求时会立即调用此方法。向进程提供动作的目标，并且应该启动恢复器的初始化或者恢复器进程。 | Yes                    |
| onCycleUpdate()    | 该方法会根据恢复器的更新速率来调用（它是一个spin函数，会被周期性调用），它应完成任何必要的更新。spin的一个例子是计算当前周期的命令速度，发布它并检查是否完成。 | Yes                    |
| onConfigure()      | 当恢复服务器进入 on_configure 状态时调用方法。理想情况下，此方法应在恢复进入配置状态（获取参数等）之前实现必要的操作。 | No                     |
| onCleanup()        | 当恢复服务器进入 on_cleanup 状态时调用方法。理想情况下，此方法应清除为恢复创建的资源。 | No                     |

本教程中将会使用onRun()、onCycleUpdate()和onConfigure()方法来创建SMS恢复器。为简单起见，将会跳过onConfigure()方法，而仅包括参数声明。

- 代码解析

  - onRun()用于设置初始化状态，并启动恢复行为，代码如下

    ```
    Status SMSRecovery::onRun(const std::shared_ptr<const Action::Goal> command)
    {
      std::string response;
      bool message_success = _twilio->send_message(
        _to_number,
        _from_number,
        command->message,
        response,
        "",
        false);
    
      if (!message_success) {
        RCLCPP_INFO(node_->get_logger(), "SMS send failed.");
        return Status::FAILED;
      }
    
      RCLCPP_INFO(node_->get_logger(), "SMS sent successfully!");
      return Status::SUCCEEDED;
    }
    ```

    在代码中，我们收到了一个action goal，名为command，需要进行处理。command中的message字段包含了我们需要同“母舰”（远端操作中心）通信的消息，即我们希望通过SMS发送给操作中心“战友”的“求救”信息。

​				我们利用服务Twillio来完成这个任务。请[创建一个帐户](https://link.zhihu.com/?target=https%3A//www.twilio.com/)并获取创建服务所需的所有相关信息（				例如account_sid、auth_token和电话号码）。可以将这些值设置为与onConfigure()参数声				明相对应的配置文件中的参数。

​				我们使用_twilio对象来发送消息，消息中包含配置文件中的你的账户信息。无论消息是否发送				成功，我们都将消息和日志显示在屏幕上。

   - - onCycleUpdate()非常简单，它就是短时运行的恢复行为的结果。如果恢复的运行时间很长，

       比如一直在spinning、正在导航前往安全区或者逃离坏点并等待救援，这个函数将会去检查超时情况或者计算控制值。

       ```
       Status SMSRecovery::onCycleUpdate()
       {
         return Status::SUCCEEDED;
       }
       ```

       这里我们直接返回SUCCEEDED即可，无需任何处理，因为我们已经在onRun中完成了任务。

## 2. 导出恢复器插件

插件是在运行时被恢复器服务器加载。在ROS2中，导出和加载插件都有库pluginlib来处理。

- 在本教程中nav2_sms_recovery::SMSRecovery插件类被加载为它的基类nav2_core::Recovery。在cpp文件末尾添加以下内容

  ```
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_sms_recovery::SMSRecovery, nav2_core::Recovery)
  ```

- 在功能包的根目录下创建插件的描述文件recovery_plugin.xml。描述文件中包含以下信息

  - `library path`: 插件库名称及其位置
  - `class name`: 类名
  - `class type`: 类的类型
  - `base class`: 基类名
  - `description`: 插件描述

  ```
  <library path="nav2_sms_recovery_plugin">
    <class name="nav2_sms_recovery/SMSRecovery" type="nav2_sms_recovery::SMSRecovery" base_class_type="nav2_core::Recovery">
      <description>This is an example plugin which produces an SMS text message recovery.</description>
    </class>
  </library>
  ```

- 修改CMakeList.txt，使用pluginlib_export_plugin_description_file()方法导出插件，它会将插件描述文件安装到share目录并设置ament索引使其可被发现

  ```
  pluginlib_export_plugin_description_file(nav2_core recovery_plugin.xml)
  ```

- 修改package.xml

  ```
  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/recovery_plugin.xml" />
  </export>
  ```

- 编译

## 3. 通过参数文件传递插件名称

为了使用插件，需要修改nav2_params.yaml文件，原始内容如下

```
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

需要替换为

```
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait", "call_for_help"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    call_for_help:
      plugin: "nav2_sms_recovery/SMSRecovery"
      account_sid: ... # your sid
      auth_token: ... # your token
      from_number: ... # your number
      to_number: ... # the operations center number
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

## 4. 运行恢复器插件

```
$ ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml
```

再开一个命令行窗口，输入

```
$ ros2 action send_goal "call_for_help" nav2_sms_recovery/action/SmsRecovery "Help! Robot 42 is being mean :( Tell him to stop!"
```



​			
