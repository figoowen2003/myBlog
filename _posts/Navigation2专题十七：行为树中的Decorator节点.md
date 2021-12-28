---
title: Navigation2专题十七：行为树中的Decorator节点
date: 2021-12-28 15:12:22
tags:
---

# 概述

装饰节点Decorators，它类似于控制节点，但是它只能拥有一个子节点。

装饰节点决定是否、何时以及tick子节点的次数。



# 分类

- InverterNode

  tick子节点一次，并且当子节点FAILURE时，它返回SUCCESS，当子节点SUCCESS时，它返回FAILURE。

  如果子节点返回RUNNING，它也返回RUNNING。

- ForceSuccessNode

  如果子节点返回RUNNING，它也返回RUNNING。

  此外，它都返回SUCCESS。

- ForceFailureNode

  如果子节点返回RUNNING，它也返回RUNNING。

  此外，它都返回FAILURE。

- RepeatNode

  只要子节点返回SUCCESS，它将tick子节点最多N次，N作为input Port传递进来。

  如果子节点返回FAILURE，中断整个循环，并返回FAILURE。

  如果子节点返回RUNNING，它也返回RUNNING。

- RetryNode

  只要子节点返回FAILURE，它将tick子节点最多N次，N作为Input Port传递进来。

  如果子节点返回SUCCESS，中断整个循环，并返回SUCCESS。

  如果子节点返回RUNNING，它也返回RUNNING。

