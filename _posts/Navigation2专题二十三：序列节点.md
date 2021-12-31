---
title: Navigation2专题二十三：序列节点
date: 2021-12-31 11:08:20
tags:
---

本小结将以具体的例子来讲解SequenceNode与ReactiveSequence之间的差别。

一个同步的动作拥有自己的线程，这意味这它允许用户使用阻塞函数，但同时也会将执行的流程返回到行为树中。

```
// Custom type
struct Pose2D
{
    double x, y, theta;
};

class MoveBaseAction : public AsyncActionNode
{
  public:
    MoveBaseAction(const std::string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    { }

	// 声明名为goal的输入端口
    static PortsList providedPorts()
    {
        return{ InputPort<Pose2D>("goal") };
    }

    NodeStatus tick() override;

    // This overloaded method is used to stop the execution of this node.
    void halt() override
    {
        _halt_requested.store(true);
    }

  private:
  	// 表示在同一时刻只有唯一的线程能够访问这个bool值，是一个原子操作
    std::atomic_bool _halt_requested;
};

//-------------------------

NodeStatus MoveBaseAction::tick()
{
    Pose2D goal;
    if ( !getInput<Pose2D>("goal", goal))
    {
        throw RuntimeError("missing required input [goal]");
    }

    printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f theta=%.2f\n", 
           goal.x, goal.y, goal.theta);

    _halt_requested.store(false);
    int count = 0;

    // Pretend that "computing" takes 250 milliseconds.
    // It is up to you to check periodicall _halt_requested and interrupt
    // this tick() if it is true.
    while (!_halt_requested && count++ < 25)
    {
        SleepMS(10);
    }

    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return _halt_requested ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
}
```

方法MoveBaseAction::tick()将在非主线程中执行，而主线程则会触发MoveBaseAction::executeTick()方法。

用户需要负责实现一个有效的halt()函数。

**Pose2D是自定义类型，用户也需要实现convertFromString<Pose2D>方法，目前有个问题，这个自定义的模板方法是定义在哪个文件中，又是何时被调用的？？**

```
// Template specialization to converts a string to Pose2D.
namespace BT
{
    template <> inline Pose2D convertFromString(StringView str)
    {
        // The next line should be removed...
        printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            Pose2D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            output.theta     = convertFromString<double>(parts[2]);
            return output;
        }
    }
} // end namespace BT
```



# Sequence VS ReactiveSequence

```
static const char* xml_text = R"(
 <root>
     <BehaviorTree>
        <Sequence>
            <BatteryOK/>
            <SaySomething   message="mission started..." />
            <MoveBase       goal="1;2;3"/>
            <SaySomething   message="mission completed!" />
        </Sequence>
     </BehaviorTree>
 </root>
 )";
 
 int main()
{
    using namespace DummyNodes;

    BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    auto tree = factory.createTreeFromText(xml_text);

    NodeStatus status;

    std::cout << "\n--- 1st executeTick() ---" << std::endl;
    status = tree.tickRoot();

    SleepMS(150);
    std::cout << "\n--- 2nd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    SleepMS(150);
    std::cout << "\n--- 3rd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    std::cout << std::endl;

    return 0;
}
```

以上为Sequence类型节点树，预期输出为

```
    --- 1st executeTick() ---
    [ Battery: OK ]
    Robot says: "mission started..."
    [ MoveBase: STARTED ]. goal: x=1 y=2.0 theta=3.00

    --- 2nd executeTick() ---
    [ MoveBase: FINISHED ]

    --- 3rd executeTick() ---
    Robot says: "mission completed!"
```

其中tree.tickRoot()就是上文提到的executeTick()，当它第一次和第二次被调用时，MoveBase节点返回RUNNING，第三次则是返回SUCCESS。

状态节点BatteryOK只执行一次。

```
 <root>
     <BehaviorTree>
        <ReactiveSequence>
            <BatteryOK/>
            <Sequence>
                <SaySomething   message="mission started..." />
                <MoveBase       goal="1;2;3"/>
                <SaySomething   message="mission completed!" />
            </Sequence>
        </ReactiveSequence>
     </BehaviorTree>
 </root>
```

更改为ReactiveSequence类型后，预期输出为

```
    --- 1st executeTick() ---
    [ Battery: OK ]
    Robot says: "mission started..."
    [ MoveBase: STARTED ]. goal: x=1 y=2.0 theta=3.00

    --- 2nd executeTick() ---
    [ Battery: OK ]
    [ MoveBase: FINISHED ]

    --- 3rd executeTick() ---
    [ Battery: OK ]
    Robot says: "mission completed!"
```

如果使用ReactiveSequence类型的节点树，那么当MoveBase返回RUNNING时，整个序列都将重启，状态节点BatteryOK会再次被执行。

如果BatteryOK节点返回FAILURE，MoveBase动作将被中断（halted）。
