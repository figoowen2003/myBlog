---
title: Navigation2专题十六：行为树中的Fallback节点
date: 2021-12-28 14:18:19
tags:
---

# 概述

Fallback节点也是控制节点的一种，在其他的框架中，该类型的节点也被成为“选择器”或者“优先级”。该类节点的目的是尝试不同的执行策略，直到我们找到一种有效的策略为止。

遵循的**规则**：

- 在Fallback节点tick第一个子节点之前，它的状态变为RUNNING
- 如果子节点返回FAILURE，会继续tick下一个子节点
- 如果最后一个子节点也返回FAILURE，所有的子节点都将终止，且Fallback返回FAILURE
- 如果一个子节点返回SUCCESS，Fallback节点将停止并返回成功，且所有子节点也将终止运行

**分类**：

| Type of ControlNode | Child returns RUNNING |
| :------------------ | --------------------- |
| Fallback            | Tick again            |
| ReactiveFallback    | Restart               |

- Restart表示整个Fallback节点将从第一个子节点开始，全部重启
- Tick again表示下一次Fallback节点被tick时，同一个子节点将被tick。该子节点的前一个已经返回成功的兄弟节点将不再被tick



# Fallback节点

以开门这个任务为例，我们将尝试不同的策略，首先检查门是否已经打开（执行一次）

![回退节点](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十六：行为树中的Fallback节点/fallbacksimplified.png)

```
    // index is initialized to 0 in the constructor
    status = RUNNING;

    while( _index < number_of_children )
    {
        child_status = child[index]->tick();

        if( child_status == RUNNING ) {
            // Suspend execution and return RUNNING.
            // At the next tick, _index will be the same.
            return RUNNING;
        }
        else if( child_status == FAILURE ) {
            // continue the while loop
            _index++;
        }
        else if( child_status == SUCCESS ) {
            // Suspend execution and return SUCCESS.
            HaltAllChildren();
            _index = 0;
            return SUCCESS;
        }
    }
    // all the children returned FAILURE. Return FAILURE too.
    index = 0;
    HaltAllChildren();
    return FAILURE;
```



# ReactiveFallback节点

当一个条件节点的状态从FAILURE转变为SUCCESS时，如果你希望中断异步的子节点，那么应该选择ReactiveFallback这种类型的控制节点。

参考下图

![反应后备](https://d33wubrfki0l68.cloudfront.net/ca7f64943113dbf26b2d406357e7f8dd9326e283/26b1e/images/reactivefallback.png)

```
    status = RUNNING;

    for (int index=0; index < number_of_children; index++)
    {
        child_status = child[index]->tick();

        if( child_status == RUNNING ) {
            // Suspend all subsequent siblings and return RUNNING.
            HaltSubsequentSiblings();
            return RUNNING;
        }

        // if child_status == FAILURE, continue to tick next sibling

        if( child_status == SUCCESS ) {
            // Suspend execution and return SUCCESS.
            HaltAllChildren();
            return SUCCESS;
        }
    }
    // all the children returned FAILURE. Return FAILURE too.
    HaltAllChildren();
    return FAILURE;
```

如果areYouRested节点返回FAILURE，则tick下一个子节点Timeout（是一个异步子节点？？？），完成sleep；如果areYouRested节点返回SUCCESS，则Timeout节点被终止，整个ReactiveFallback节点也终止，并返回SUCCESS。
