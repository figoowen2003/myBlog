---
title: Navigation2专题二十二：通用端口
date: 2021-12-30 18:24:47
tags:
---

本节将讲述如何在端口中使用C++通用类型



# 解析string

BehaviourTree.CPP支持自动将string转换为通用类型如int、long、double、bool、NodeStatus等。

也可以支持用户自定义类型，如

```
// We want to be able to use this custom type
struct Position2D 
{ 
  double x;
  double y; 
};
```

为了将string解析为Position2D类型，我们需要使用以下模板方法

**BT::convertFromString<Position2D>(StringView)**

该方法的实现如下

```
// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline Position2D convertFromString(StringView str)
    {
        // The next line should be removed...
        printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            Position2D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            return output;
        }
    }
} // end namespace BT
```

- StringView是std::string_view的C++11版本，它可以接收一个std::string或者const char*。
- BehaviourTree.CPP库提供了splitString函数，也可以使用boost::algorithm::split。
- 一旦我们将输入分割成单个数字，我们就可以复用convertFromString<double>()





# Example

我们创建两个自定义动作，一个将写一个端口，另一个将读这个端口

```
class CalculateGoal: public SyncActionNode
{
public:
    CalculateGoal(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name,config)
    {}

    static PortsList providedPorts()
    {
        return { OutputPort<Position2D>("goal") };
    }

    NodeStatus tick() override
    {
        Position2D mygoal = {1.1, 2.3};
        setOutput<Position2D>("goal", mygoal);
        return NodeStatus::SUCCESS;
    }
};


class PrintTarget: public SyncActionNode
{
public:
    PrintTarget(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name,config)
    {}

    static PortsList providedPorts()
    {
        // Optionally, a port can have a human readable description
        const char*  description = "Simply print the goal on console...";
        return { InputPort<Position2D>("target", description) };
    }

    NodeStatus tick() override
    {
        auto res = getInput<Position2D>("target");
        if( !res )
        {
            throw RuntimeError("error reading port [target]:", res.error());
        }
        Position2D target = res.value();
        printf("Target positions: [ %.1f, %.1f ]\n", target.x, target.y );
        return NodeStatus::SUCCESS;
    }
};
```

我们可以通过将指向Blackboard同一条目的方式，将输入与输出端口连接起来。

我们创建了一棵有四个动作节点的序列节点树

- 使用动作CalculateGoal将Position2D的值存储在条目“GoalPosition”中
- 调用PrintTarget，输入“target”将读取来自Blackboard条目“GoalPosition”的内容
- 使用内置操作SetBlackboard写key值“OtherGoal”，一个从string到Position2D的转换将在后台完成
- 调用PringTarget，输入“target”将读取来自Blackboard条目“OtherGoal”的内容

```
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <SequenceStar name="root">
            <CalculateGoal   goal="{GoalPosition}" />
            <PrintTarget     target="{GoalPosition}" />
            <SetBlackboard   output_key="OtherGoal" value="-1;3" />
            <PrintTarget     target="{OtherGoal}" />
        </SequenceStar>
     </BehaviorTree>
 </root>
 )";

int main()
{
    using namespace BT;

    BehaviorTreeFactory factory;
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");

    auto tree = factory.createTreeFromText(xml_text);
    tree.tickRoot();

/* Expected output:

    Target positions: [ 1.1, 2.3 ]
    Converting string: "-1;3"
    Target positions: [ -1.0, 3.0 ]
*/
    return 0;
}
```

