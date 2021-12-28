---
title: Navigation2专题十四：行为树中的基本概念
date: 2021-12-27 17:25:13
tags:
---

- BehaviorTree.CPP：是一个C++库，可以集成到分布式的middleware中，比如ROS或者SmartSoft。

- Node vs Tree

  用户需要创建自定义功能的动作节点和状态节点（ActionNode、ConditionNodes），这些节点都是叶子节点。BehaviorTree.CPP库会帮助用户将自定义叶子节点组合到树中。

  自定义节点应该是高度可重用的。

- tick（）callback

  任何的树节点都可以看作是一种触发回调的机制，回调的行为和内容取决于开发者的意图。

- inheritance vs dependency injection（继承与依赖注入）

  为了创建自定义节点，你需要继承自适当的基类。比如创建你自己的同步动作节点，需要继承基类SyncActionNode。

  依赖注入则是BehaviorTree.CPP库提供的另一种自定义节点的机制，它需要将一个函数指针传递给相关的包装器。

​		具体实现会在之后慢慢展开。

- Dataflow, Ports and Blackboard

  Blackboard是一棵树中所有节点可以共享的键/值的存储器。

  Ports是节点之间交换信息的一种机制。节点是通过Blackboard中相同的键连接起来的。

  一个节点的Ports数量、名称及类型都是要在编译时就明确（C++）；而Ports间的连接则是在部署时完成（XML）。

- Load trees at run-time using the XML format

  行为树本身可以在运行时再进行组合，具体的说是在部署时组合，通过XML文件在开始阶段完成一次树的实例化。

  详细内容会在之后介绍。









​		
