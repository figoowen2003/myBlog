---
title: Navigation2专题二十七：传递额外参数
date: 2021-12-31 16:33:02
tags:
---

在之前的每个例子中，我们都“被迫”提供具有以下函数签名的构造函数

```
    MyCustomNode(const std::string& name, const NodeConfiguration& config);
```

在同样的情况下，我们希望能够向构造函数传递额外的参数，比如普通形参、指针、引用等等。

**很多开发者会使用Blackboard来实现参数的传递，但是并不推荐这种方式。**

理论上讲，这些参数是可以通过输入端口来传递的，但是在以下条件下，这样做是错误的：

- 这些参数在部署时被感知到
- 这些参数在运行时不发生变化
- 这些参数不必在XML文件中配置

如果以上三点都满足，那么使用端口或者Blackboard就是繁琐和不必要的。



# 方法一：注册一个自定义构建器

假设自定义的节点为Action_A，我们希望传递三个参数，它们可以是任意复杂的对象，也不限制与内置类型

```
// Action_A has a different constructor than the default one.
class Action_A: public SyncActionNode
{

public:
    // additional arguments passed to the constructor
    Action_A(const std::string& name, const NodeConfiguration& config,
             int arg1, double arg2, std::string arg3 ):
        SyncActionNode(name, config),
        _arg1(arg1),
        _arg2(arg2),
        _arg3(arg3) {}

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    // tick() can access the private members
    NodeStatus tick() override;

private:
    int _arg1;
    double _arg2;
    std::string _arg3;
};
```

因为构造函数的问题，这个节点不能直接被注册，需要定义一个构造器来封装这个节点的实例。此处用lambda表达式来封装节点的实例化过程。

```
BehaviorTreeFactory factory;

// A node builder is a functor that creates a std::unique_ptr<TreeNode>.
// Using lambdas or std::bind, we can easily "inject" additional arguments.
NodeBuilder builder_A =
   [](const std::string& name, const NodeConfiguration& config)
{
    return std::make_unique<Action_A>( name, config, 42, 3.14, "hello world" );
};

// BehaviorTreeFactory::registerBuilder is a more general way to
// register a custom node.
factory.registerBuilder<Action_A>( "Action_A", builder_A);

// Register more custom nodes, if needed.
// ....

// The rest of your code, where you create and tick the tree, goes here.
// ....
```



# 方法二：使用init方法

```
class Action_B: public SyncActionNode
{

public:
    // The constructor looks as usual.
    Action_B(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name, config) {}

    // We want this method to be called ONCE and BEFORE the first tick()
    void init( int arg1, double arg2, const std::string& arg3 )
    {
        _arg1 = (arg1);
        _arg2 = (arg2);
        _arg3 = (arg3);
    }

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    // tick() can access the private members
    NodeStatus tick() override;

private:
    int _arg1;
    double _arg2;
    std::string _arg3;
};
```

在节点类中定义init成员函数，来完成额外参数的初始化。

注册和初始化方法如下

```
BehaviorTreeFactory factory;

// The regitration of  Action_B is done as usual, but remember
// that we still need to call Action_B::init()
factory.registerNodeType<Action_B>( "Action_B" );

// Register more custom nodes, if needed.
// ....

// Create the whole tree
auto tree = factory.createTreeFromText(xml_text);

// Iterate through all the nodes and call init() if it is an Action_B
for( auto& node: tree.nodes )
{
    // Not a typo: it is "=", not "=="此处是一个强转，不是做条件判断
    if( auto action_B = dynamic_cast<Action_B*>( node.get() ))
    {
        action_B->init( 42, 3.14, "hello world");
    }
}

// The rest of your code, where you tick the tree, goes here.
// ....
```



