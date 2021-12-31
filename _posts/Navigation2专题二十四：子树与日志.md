---
title: Navigation2专题二十四：子树与日志
date: 2021-12-31 14:50:19
tags:
---

通过将较小的、可复用的行为树组合成较大的树的方式，我们可以创建大尺度的行为树。换句话说，我们希望创建分层的行为树。

只需在XML文件中定义多个行为树，并且用其中一棵去包含另外的行为树，就可以构建分层的行为树。



# 嵌套行为

下面是一个很好的嵌套子树的例子，其中还使用了Decorator节点与Fallback节点。

```
<root main_tree_to_execute = "MainTree">

    <BehaviorTree ID="DoorClosed">
        <Sequence name="door_closed_sequence">
            <Inverter>
                <IsDoorOpen/>
            </Inverter>
            <RetryUntilSuccesful num_attempts="4">
                <OpenDoor/>
            </RetryUntilSuccesful>
            <PassThroughDoor/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="MainTree">
        <Fallback name="root_Fallback">
            <Sequence name="door_open_sequence">
                <IsDoorOpen/>
                <PassThroughDoor/>
            </Sequence>
            <SubTree ID="DoorClosed"/>
            <PassThroughWindow/>
        </Fallback>
    </BehaviorTree>

</root>
```

在这个例子当中，主树为MainTree，子树为DoorClosed。这棵行为树希望实现的行为：

- 如果门是打开的，那么PassThroughDoor。
- 如果门是关闭的，尝试4次OpenDoor，如果成功，那么PassThroughDoor。
- 如果无法打开关闭着的门，PassThroughWindow。



# 日志

日志是显示，记录以及发布行为树状态变化的一种机制。

```
int main()
{
    using namespace BT;
    BehaviorTreeFactory factory;

    // register all the actions into the factory
    // We don't show how these actions are implemented, since most of the 
    // times they just print a message on screen and return SUCCESS.
    // See the code on Github for more details.
    factory.registerSimpleCondition("IsDoorOpen", std::bind(IsDoorOpen));
    factory.registerSimpleAction("PassThroughDoor", std::bind(PassThroughDoor));
    factory.registerSimpleAction("PassThroughWindow", std::bind(PassThroughWindow));
    factory.registerSimpleAction("OpenDoor", std::bind(OpenDoor));
    factory.registerSimpleAction("CloseDoor", std::bind(CloseDoor));
    factory.registerSimpleCondition("IsDoorLocked", std::bind(IsDoorLocked));
    factory.registerSimpleAction("UnlockDoor", std::bind(UnlockDoor));

    // Load from text or file...
    auto tree = factory.createTreeFromText(xml_text);

    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    // This logger saves state changes on file
    FileLogger logger_file(tree, "bt_trace.fbl");

    // This logger stores the execution time of each node
    MinitraceLogger logger_minitrace(tree, "bt_trace.json");

#ifdef ZMQ_FOUND
    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree);
#endif

    printTreeRecursively(tree.rootNode());

    //while (1)
    {
        NodeStatus status = NodeStatus::RUNNING;
        // Keep on ticking until you get either a SUCCESS or FAILURE state
        while( status == NodeStatus::RUNNING)
        {
            status = tree.tickRoot();
            CrossDoor::SleepMS(1);   // optional sleep to avoid "busy loops"
        }
        CrossDoor::SleepMS(2000);
    }
    return 0;
}
```

