---
title: Navigation2专题十三：Behavior Trees简介
date: 2021-12-27 10:18:09
tags:
---

# 概念

**Behavior Tree**：行为树，它是一种分层的节点树，用于控制决策流和“任务”的执行，我们也可以将“任务”进一步理解为“动作”。

**leaves**：树的叶子节点是实际的命令，即我们的协调组件与系统其余部分交互的地方。

​				举个例子，在一个面向服务的架构中，叶子节点包含了“客户端”代码，通过该代码与执行操作				的“服务器”通信。

在下面的例子中，将看到两个按顺序执行的动作：DetectObject与GraspObject。

![叶到组件通信](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十三：Behavior-Trees简介/leaftocomponentcommunication.png)

行为树的非叶子节点，则是控制“**执行流程**”。

“**执行流程**”可以这样来理解：想象一个“tick”信号，它从树的根部开始执行，并通过分支传播，直到到达一个或多个叶子节点。tick在这里表示触发一个TreeNode的回调函数tick()。

当一个TreeNode被tick时，将返回以下任意一种节点状态：

- SUCCESS
- FAILURE
- RUNNING 当异步节点的执行过程没有完成，仍然需要更多的时间去返回一个有效结果时，异步节点就会返回RUNNING状态；当然异步节点可以被终止。

节点的结果将被传回它的父节点，这个结果将决定接下来tick哪个子节点或者这个结果是否要继续返回给它的父节点。



# 节点类型

- **ControlNodes**：控制节点拥有一个或多个子节点。一旦它收到一个tick，该tick将被传递到一个或多个子节点中。
- **DecoratorNodes**：装饰节点，类似于控制节点，但是它只能拥有一个子节点。
- **ActionNodes**：动作节点为叶子节点，用户应该实现自己的ActionNode来执行实际任务。
- **ConditionNodes**：等同于动作节点，它也是叶子节点，但是该节点一直保持原子状态和同步状态，如他们不能返回RUNNING状态。他们也不应该改变系统的状态。

![UML hierarchy](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十三：Behavior-Trees简介/typehierarchy.png)



# 举例说明

我们将给出实际的例子来帮助理解行为树的工作过程，为简单起见，我们将不考虑一个动作返回RUNNING状态的情况，并假设每个动作都是以原子和同步的方式执行的。

- 第一个控制节点：Sequence

  Sequence是一个使用频率最高的最基础的控制节点，也称为SequenceNode，序列节点。该节点的子节点都是有序执行的，在图形表示中，执行的顺序为从左到右。

  下图是取一杯啤酒的一个流程

  ![Simple Sequence: fridge](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十三：Behavior-Trees简介/sequencebasic.png)

  - 如果子节点返回SUCCESS，则tick其右侧的子节点
  - 如果子节点返回FAILURE，则不再tick下一个子节点，整个Sequence返回FAILURE
  - 如果所有子节点都返回SUCCESS，则Sequence也返回SUCCESS
  - 但是，如上图所示，假设GrabBeer节点失败，那么冰箱将维持打开状态，因为跳过了CloseFridge这个节点。

- 装饰节点

  根据装饰节点的类型，它的目的可以包括

  - 转换它从其子节点收到的结果
  - 停止其子节点的执行过程
  - 反复地去tick其子节点

  下图为装饰节点的一种使用方式

  ![Simple Decorator: Enter Room](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十三：Behavior-Trees简介/decoratorenterroom.png)

  装饰节点**Inverter**，顾名思义表示它将反转其子节点的返回结果，因此上图中Inverter后面跟有名为DoorOpen的节点，Inverter + DoorOpen 等价于

  ```
  Is the door closed?
  ```

  图中流程含义为：

  如果**Inverter**的子节点返回FAILURE，Inverter将其反转为SUCCESS，表示门是关闭的，则**Retry**节点将尝试tick其子节点执行开门动作，如果开门节点返回FAILURE，将Retry节点将重试tick其子节点OpenDoor总共3次，超过3次则放弃尝试并返回FAILURE。

  注意，如果Inverter的子节点返回SUCCESS，则Inverter将其反转为FAILURE，那么表示门是开的，则整个Inverter返回FAILURE，导致整个Sequence被中断。

- 第二个控制节点：Fallback

  **FallbackNodes**：退路节点/回退节点，也称“选择器”，是可以表达回退策略的节点，即子节点返回FAILURE时下一步该做什么。

  它按顺序tick其子节点，遵循以下原则：

  - 如果当前子节点返回FAILURE，则tick下一个子节点
  - 如果当前子节点返回SUCCESS，则不再tick下一个，且Fallback返回SUCCESS
  - 如果所有子节点都返回FAILURE，则Fallback返回FAILURE

  下图将展示Sequence节点与Fallback节点是如何结合的

  ![FallbackNodes](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十三：Behavior-Trees简介/fallbackbasic.png)

​		门是否打开？

​		如果不是，尝试开门；

​		如果开门失败，如果你有钥匙，解锁并开门；

​		如果失败，砸碎那把锁；

​		如果上述任何一个动作SUCCESS，那么就可以进入房间；否则，失败。

- 重新考察“去啤酒”的例子

  使用“绿色”表示返回成功的节点，使用“红色”表示返回失败的节点，“黑色”节点表示未执行。

  ![获取啤酒失败](https://d33wubrfki0l68.cloudfront.net/889e88af56d79fad6fec87ace84087d3cbbc7fbe/c636d/images/fetchbeerfails.png)

​		我们创建一棵替代树来解决上图中遗留的问题，即便取啤酒失败，也会关上冰箱门。

​		![FetchBeer failure](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十三：Behavior-Trees简介/fetchbeer.png)

​		左侧的树会一直返回SUCCESS，无论是否取到啤酒；

​		右侧的树则意味着如果取到啤酒，则Fallback返回SUCCESS，关闭冰箱门，如果没有取到啤酒，则		进入ForceFailure节点，关上冰箱门，强制返回Failure。
