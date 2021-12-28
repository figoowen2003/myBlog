---
title: Navigation2专题十五：行为树中的Sequence节点
date: 2021-12-28 10:26:40
tags:
---

# 概述

Sequence代表行为树中的序列节点，是控制节点的一种，它拥有一个或多个子节点。当所有的子节点都返回SUCCESS时，它就会tick全部的子节点。如果任一子节点返回FAILURE，序列节点就会终止。

序列节点遵循的**规则**如下

- 在tick第一个子节点之前，节点的状态变为RUNNING。（此处费解，是哪个节点的状态变为RUNNING，我理解为是序列节点状态变为RUNNING）
- 如果一个子节点返回SUCCESS，序列节点将tick下一个子节点
- 如果最后一个子节点也返回SUCCESS，所有的子节点都将终止，且序列节点返回SUCCESS

**序列节点的分类**

| Type of ControlNode | Child returns FAILURE | Child returns RUNNING |
| :------------------ | --------------------- | --------------------- |
| Sequence            | Restart               | Tick again            |
| ReactiveSequence    | Restart               | Restart               |
| SequenceStar        | Tick again            | Tick again            |

- Restart表示整个序列节点姜葱第一个子节点开始，全部重启
- Tick again表示下一次序列节点被tick时，同一个子节点将被tick。该子节点的前一个已经返回成功的兄弟节点将不再被tick



# Sequence

以电脑中的射击游戏为例

![SequenceNode](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十五：行为树中的Sequence节点/sequencenode.png)

```
    status = RUNNING;
    // _index is a private member

    while(_index < number_of_children)
    {
        child_status = child[_index]->tick();

        if( child_status == SUCCESS ) {
            _index++;
        }
        else if( child_status == RUNNING ) {
            // keep same index
            return RUNNING;
        }
        else if( child_status == FAILURE ) {
            HaltAllChildren();
            _index = 0;
            return FAILURE;
        }
    }
    // all the children returned success. Return SUCCESS too.
    HaltAllChildren();
    _index = 0;
    return SUCCESS;
```



# ReactiveSequence

该节点类型特别适用于状态检测；但是使用异步子节点时也需小心，得确保异步子节点被tick的次数不要超过逾期次数。

![ReactiveSequence](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十五：行为树中的Sequence节点/reactivesequence.png)

其中ApproachEnemy就是一个会持续返回RUNNING的异步动作子节点，除非它真的完成了任务。

状态子节点isEnemyVisible将被多次调用，如果它返回false（FAILURE），ApproachEnemy子节点将会终止。(似乎和表格中描述不服，不是应该全部重启吗，为何是终止)

```
    status = RUNNING;

    for (int index=0; index < number_of_children; index++)
    {
        child_status = child[index]->tick();

        if( child_status == RUNNING ) {
            return RUNNING;
        }
        else if( child_status == FAILURE ) {
            HaltAllChildren();
            return FAILURE;
        }
    }
    // all the children returned success. Return SUCCESS too.
    HaltAllChildren();
    return SUCCESS;
```



# SequenceStar

当你不希望再次tick已经返回SUCCESS的子节点时，应该使用这个类型的序列节点。

以巡逻机器人为例，它需要依次方位位置A，B，C。如果动作GoTo(B)失败，那么GoTo(A)也不会再被tick。

![SequenceStar](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十五：行为树中的Sequence节点/sequencestar.png)

如上图所示，isBatteryOk必须在每次tick前被检查，因此该SequenceStar节点还需要一个ReactiveSequence的父节点。

```
    status = RUNNING;
    // _index is a private member

    while( index < number_of_children)
    {
        child_status = child[index]->tick();

        if( child_status == SUCCESS ) {
            _index++;
        }
        else if( child_status == RUNNING || 
                 child_status == FAILURE ) 
        {
            // keep same index
            return child_status;
        }
    }
    // all the children returned success. Return SUCCESS too.
    HaltAllChildren();
    _index = 0;
    return SUCCESS;
```

