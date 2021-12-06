---
title: Navigation2专题十一：编写一个新的行为树插件
date: 2021-12-06 10:02:45
tags:
---

行为树插件（Behaviour Tree Plugin）会被用作行为树XML文件中的节点来使用，BT Navigator来处理这个XML文件用于提供导航逻辑。

# 要求

在本地机器上已经安装或构建了以下软件包：

● ROS 2（[二进制](https://www.zhihu.com/search?q=二进制&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"article"%2C"sourceId"%3A389458841})安装或从源代码构建）

● Nav2（包括依赖包）

● Gazebo

● Turtlebot3



# 步骤

## 1. 创建一个新的BT插件

本教程中将以nav2_behaviour_tree功能包中的behaviour tree action节点--名为wait的节点为例，来分析最简单的行为树功能。除此之外，我们也可以去创建自定义的decorator，condition以及control节点，这些节点分别代表这行为树中独一无二的表现行为如规划，行为树的流控制，一个条件状态的检查，或者修改其他BT节点的输出等动作。

代码连接[nav2_behavior_tree](https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree) ，参考wait_action节点。

教程中的插件继承自基类nav2_behaviour_tree::BtActionNode。这个基类对BehaviorTree.cpp中的BT::ActionNodeBase做了封装，这样就可以使用ROS2的action客户对来操作BT action。一个BtActionNode既是一个BT的action，同时也是ROS2的action，可以使用ROS2的action网络接口去调用远程的服务去完成任务。

如果使用其他类型的BT节点（如decorator，control，condition），则需要继承相应的BT node，BT::DecoratorNode、BT::ControlNode、 BT::ConditionNode。如果只使用BT action节点而无需ROS2 action的接口，则继承BT::ActionNodeBase即可。

除了构造函数中的信息外，BTActionNode类还提供了5个虚函数供用户完成其功能。

| ** 方法说明**        | **方法描述**                                                 | 是否必须 |
| -------------------- | ------------------------------------------------------------ | -------- |
| 构造函数Constructor  | 构造函数用于指示相关XML文件中与插件匹配的标记名称、调用插件的action服务器的名称以及其他所需的 BehaviorTree.CPP 指定的配置信息。 | 是       |
| providedPorts()      | 定义 BT 节点可能具有的输入和输出端口。这些端口类似于在 BT XML 文件中通过硬编码的值定义的参数，或由其他节点的输出端口的值定义的参数。 | 是       |
| on_tick()            | 当这个BT节点在执行过程中被行为树标记时，这个方法会被调用。该方法用于获取动态更新，例如新的黑板值、输入端口或参数。也可以重置action的状态。 | 不       |
| on_wait_for_result() | 当行为树节点正在等待它调用的 ROS 2 动作服务器的返回结果时，调用该方法。它用于检查更新以抢占当前任务、检查超时或在等待操作完成时去计算的任何内容。 | 不       |
| on_success()         | 当 ROS 2 动作服务器返回成功结果时调用该方法。返回值将由BT节点回报给行为树。 | 不       |
| on_aborted()         | 当 ROS 2 动作服务器返回中止结果时调用该方法。返回值将由BT节点回报给行为树。 | 不       |
| on_cancelled()       | 当 ROS 2 动作服务器返回取消的结果时调用该方法。返回值将由BT节点回报给行为树。 | 不       |

本教程中，值使用了on_tick()方法。

以下为构造函数，我们需要获取任何应用于行为树节点的非可变参数，如从行为树XML的输入端口获取sleep的持续时间。

```
WaitAction::WaitAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf)
{
  int duration;
  getInput("wait_duration", duration);
  if (duration <= 0) {
    RCLCPP_WARN(
      node_->get_logger(), "Wait duration is negative or zero "
      "(%i). Setting to positive.", duration);
    duration *= -1;
  }

  goal_.time.sec = duration;
}
```

xml_tag_name：Bt node插件在行为树XML中对应的名字

action_name：调用的action服务器的名称

conf：一系列配置的集合，大多数节点插件可以忽略这些配置

这个构造函数在调用时需要用ROS2的action类型类完成模板化过程，该类型为nav2_msgs::action::Wait。当该节点被行为树调用时，行为树会直接调用BTActionNode的tick()函数，随后该节点的on_tick()函数会伴随action客户端的goal一起被调用。

在构造函数中，我们获取的参数wait_duration的输入值，这个值可以在行为树中为每个wait节点的实例单独去配置。这个值保存在duration变量中，并作为goal_变量的成员。goal_变量会作为ROS2 action的goal发送到action server。本例中，duration就是想要等待的时间。

以下为providedPorts()函数，它用来定义输入或者输出端口。

```
static BT::PortsList providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<int>("wait_duration", 1, "Wait time")
    });
}
```

端口可以理解为行为树节点所访问的来自行为树本身的参数。如本例中，只有一个输入端口wait_duration，可以在行为树BT的XML文件中为每个wait恢复器插件实例设置这个端口。在函数providedPorts中，我们设置了类型int，默认值1，端口名wait_duration和端口描述信息wait time。

接下来是on_tick()方法，当行为树标记一个特定节点时会调用该方法

```
void WaitAction::on_tick()
{
  increment_recovery_count();
}
```

对于wait BT节点而言，我们只想简单地去通知黑板上的一个计数器，一个恢复器对应的action插件被标记了。这个有助于保存某次导航运行期间执行的恢复器数量。

## 2. 导出插件

导出插件是为了当一个行为树加载自定义BT XML文件时，该插件是可见的。插件将在运行时加载，但如果插件不可见，我们的BT Navigator服务器将不能加载或使用它。在BehaviorTree.CPP文件中，由宏BT_REGISTER_NODES来完成插件的导出和加载。

```
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::WaitAction>(name, "wait", config);
    };

  factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);
}
```

在这个宏中，必须创建一个NodeBuilder以便自定义的action节点能够具有非默认构造函数的签名（不太好懂，目的是用于action和xml的名称）。NodeBuilder由一个lambda表达式返回一个unique ptr去指向我们所创建的节点。需要为构造函数提供其所需要的name和config参数，接着定义这个BT节点将会调用的ROS2动作服务器名称，本例中为"wait" action，类型为nav2_behaviour_tree::WaitAction。最后将builder交给一个factory进行注册，factory注册函数中的Wait是行为树XML文件中对应这个BT节点插件的名称。如下所示，名为Wait的BT XML节点指定了一个不可变输入端口wait_duration为5s。

```
<Wait wait_duration="5"/>
```

## 3. 将插件库的名称添加到配置中

在bt_navigator节点的yaml格式配置文件中，列出插件库的名字。如下所示，请注意plugin_lib_names参数下的nav2_wait_action_bt_node

```
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_back_up_action_bt_node # other plugin
    - nav2_wait_action_bt_node    # our new plugin
```

## 4. 运行自定义插件

以navigate_w_replanning_and_recovery.xml文件为例，这个xml就可以视为一棵行为树

```
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <SequenceStar name="RecoveryActions">
          <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
        </SequenceStar>
      </ReactiveFallback>
    </RecoveryNode>
  </Behavi
```

在NavigateToPose的指定导航请求中选择这个BT XML文件，或者将此文件作为BT Navigator的yaml配置文件中的默认行为树。
