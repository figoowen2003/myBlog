---
title: Navigation2专题二十五：端口重映射
date: 2021-12-31 15:33:21
tags:
---

端口重映射是指在一棵树与子树之间的端口重映射。在上一小节例子中，一棵子树从它父节点的角度来看，就如同一个单一的叶子节点一般。

在一棵庞大的行为树中，为了避免名称冲突，任一颗树与其子树都需要使用Blackboard的不同实例，因此我们需要显示的将一棵树的端口与它子树的端口连接起来。

这就要用到端口的重映射技术，它只需在XML中进行定义就可以实现。



# 例子

考察下面的行为树

```
<root main_tree_to_execute = "MainTree">

    <BehaviorTree ID="MainTree">

        <Sequence name="main_sequence">
            <SetBlackboard output_key="move_goal" value="1;2;3" />
            <SubTree ID="MoveRobot" target="move_goal" output="move_result" />
            <SaySomething message="{move_result}"/>
        </Sequence>

    </BehaviorTree>

    <BehaviorTree ID="MoveRobot">
        <Fallback name="move_robot_main">
            <SequenceStar>
                <MoveBase       goal="{target}"/>
                <SetBlackboard output_key="output" value="mission accomplished" />
            </SequenceStar>
            <ForceFailure>
                <SetBlackboard output_key="output" value="mission failed" />
            </ForceFailure>
        </Fallback>
    </BehaviorTree>

</root>
```

- 主树MainTree包含一个子树MoveRobot。
- 我们希望将MoveRobot中端口与MainTree的其他端口连接（映射）起来。

下图显示的两颗树之间的重映射关系

![ports remapping](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题二十五：端口重映射/t06_remapping.png)

- MainTree端口move_goal被映射为MoveRobot的target
- MainTree端口move_result被映射为MoveRobot的output

处于调试的目的，我们也可以使用debugMessage()方法来显示一个Blackboard的当前状态信息

```
int main()
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MoveBaseAction>("MoveBase");

    auto tree = factory.createTreeFromText(xml_text);

    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while( status == NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
        SleepMS(1);   // optional sleep to avoid "busy loops"
    }

    // let's visualize some information about the current state of the blackboards.
    std::cout << "--------------" << std::endl;
    tree.blackboard_stack[0]->debugMessage();
    std::cout << "--------------" << std::endl;
    tree.blackboard_stack[1]->debugMessage();
    std::cout << "--------------" << std::endl;

    return 0;
}

/* Expected output:

    [ MoveBase: STARTED ]. goal: x=1 y=2.0 theta=3.00
    [ MoveBase: FINISHED ]
    Robot says: mission accomplished
    --------------
    move_result (std::string) -> full
    move_goal (Pose2D) -> full
    --------------
    output (std::string) -> remapped to parent [move_result]
    target (Pose2D) -> remapped to parent [move_goal]
    --------------
*/
```

