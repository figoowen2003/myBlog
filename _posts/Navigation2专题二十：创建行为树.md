---
title: Navigation2专题二十：创建行为树
date: 2021-12-29 10:14:05
tags:
---

行为树类似于状态机，是一种在正确的时间正确的条件下触发回调的机制。而回调函数中会发生什么取决与开发人员。在本教程中，大部分时间内，我们设计的动作Actions只会实现一些打印信息，来模拟真正的任务。



# 创建自定义ActionNode

创建树节点的默认方法是通过**继承**（也是**推荐**的方法）

```
// Example of custom SyncActionNode (synchronous action)自定义同步动作
// without ports. 无端口
class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
```

- 树节点的任何实例都有一个name，通过构造函数来传递，该标识符主要供开发者阅读，它并不需要是唯一的。
- 方法tick()是实际动作发生的地方，它必须始终返回NodeStatus，即RUNNING、SUCCESS或FAILURE。

另一种创建树节点的方式是使用**依赖注入**

需要给定一个函数指针（functor）

```
    BT::NodeStatus myFunction()
    BT::NodeStatus myFunction(BT::TreeNode& self) 
```

满足上述两种函数签名之一的functor都可以创建出一个树节点

```
using namespace BT;

// Simple function that return a NodeStatus 无参数的简单形式，也需要返回NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// We want to wrap into an ActionNode the methods open() and close()
// 也可以将方法open(), close()封装到一个动作节点中，节点是一个名为GripperInterface的类
class GripperInterface
{
public:
    GripperInterface(): _open(true) {}

    NodeStatus open() {
        _open = true;
        std::cout << "GripperInterface::open" << std::endl;
        return NodeStatus::SUCCESS;
    }

    NodeStatus close() {
        std::cout << "GripperInterface::close" << std::endl;
        _open = false;
        return NodeStatus::SUCCESS;
    }

private:
    bool _open; // shared information
};
```

我们可以这些functors中任选一个构建一个SimpleActionNode

- CheckBattery()
- GripperInterface::open()
- GripperInterface::close()



# 使用XML动态创建一棵树

我们新建一个xml文件，命名为my_tree.xml，其内容如下

```
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="check_battery"/>
            <OpenGripper    name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>
 </root>
```

行为树为MainTree，根节点为root_sequence，它按顺序包含CheckBattery、OpenGripper、ApproachObject及CloseGripper四个子节点。

接下来，我们需要将自定的树节点注册到BehaviorTreeFactory中，然后从文件或文本中加载XML。

**XML中使用的标识符必须与用于注册的树节点的标识符一致。**

XML中“name”代表实例的名称；它是可选的。

以下是如何注册我们的树节点

```
#include "behaviortree_cpp_v3/bt_factory.h"

// file that contains the custom nodes definitions
#include "dummy_nodes.h"

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes BehaviorTreeFactory由
    BehaviorTreeFactory factory;

    // Note: the name used to register should be the same used in the XML.
    using namespace DummyNodes;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));

    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("./my_tree.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/
```

- 如果树节点是一个派生类，那么使用registerNodeType模板函数来注册
- 如果树节点是一个functor，且是一个条件节点，则使用registerSimpleCondition注册
- 如果树节点是一个封装到类中的成员函数functor，则使用registerSimpleAction注册
- 注册完毕后，需要加载XML文件来生成行为树
- 最后条用行为树的tickRoot()方法，来触发叶子节点的行为
