---
title: ROS2中的launch系统
date: 2021-05-20 15:13:37
tags: ROS2，launch，python
---

# 简介

## 概要

ROS2中launch系统的职责是帮助用户描述他们的系统配置（这里可以理解为ros中的package），并且按照配置去执行该系统。

系统配置主要包括以下几点：

- 要运行什么程序
- 在哪里运行这些程序
- 在程序之间应该传递什么样的参数
- ros的特殊约定，这些约定通过赋予系统中每个组件不同的配置，使得在整个系统中复用组件变得容易起来

launch还负责监视进程启动的状态，对这些进程状态的变化进行报告或做出反应。

##  launch文件中的关键词

- action：翻译为动作。执行一个进程，或者包含另外一个launch description等动作都属于action，它用于表达用户的意图。action可以产生直接的作用（例如运行进程或设置变量），同时它也可以产生其他的action。动作是可以带参数的，参数能够影响action的行为，这些参数位于‘launch.Substitution’类被使用的地方。

- Launch Entity：翻译为启动实体，launch系统中最主要的对象就是‘launch.LaunchDescriptionEntity’这个类，其他的启动实体都从这个实体中继承。这个类或者从这个类派生的子类负责捕获系统架构者有关如何启动这个系统的意图，以及启动本身如何应对启动期间的异步事件（不太明白）。

  当实体被访问时，这些实体可能会产生其他需要被访问的实体，可以从‘root’实体开始使用这个模式，此时launch系统提供了名为‘Launch.LaunchDescription’的类来开始这个启动的过程 。
  
- Base Action：‘launch.Action’是所有action的基类，它提供了Launch系统中一些公共的接口。base action是一个启动描述（文件）中的第一个类元素，它也继承了‘launch.LaunchDescriptionEntity’。action可以和条件结合起来，如‘launch.IfCondition’类，‘launch.UnlessCondition’类，以此来决定这个action是否执行。action也可以传递参数。

- Substitution：它有点类似与C++中的模板参数，在launch文件被执行之前，我们都无法得知它究竟表示的是什么。目前已经存在很多substitution的变体，它们都继承自‘launch.Substitution’这个类。

- The Launch Service：启动服务负责处理已发送的事件，将它们分发给事件处理程序（句柄），并根据需要执行action。Launch service提供了三种主要服务：

  - 包含一个launch description供任何线程调用
  - 运行事件循环
  - 关闭：取消任何正在运行的action和事件程序；终止正在运行的事件循环；能被任何线程调用

- Event Handlers：事件处理程序（句柄），它由名为‘launch.EventHandler’的基类来表示。它定义了两种主要的方法：

  - ‘launch.EventHandler.matches’：将事件作为输入，如果事件句柄和事件匹配返回true，反之false
  - ‘launch.EventHandler.handle’：将事件和launch上下文作为输入，可以选择返回一个被launch service访问的‘launch.LaunchDescriptionEntity’对象的列表。

  它并不是继承自‘launch.LaunchDescriptionEntity’，但是能够被launch service所访问。