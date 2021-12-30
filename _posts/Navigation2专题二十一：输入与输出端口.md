---
title: Navigation2专题二十一：输入与输出端口
date: 2021-12-30 16:25:43
tags:
---

如上一节所述，自定义TreeNode可用于执行任意简单或复杂的软件，它们的目标是提供一个更高抽象级别的接口。因此TreeNode（树节点）在概念上与函数并无差别。

类似与使用函数，通常我们会有如下诉求：

- 将参数传递给节点（inputs）
- 从一个节点获取某种信息（outputs）
- 一个节点的输出可能是另一个节点的输入

BehaviorTree.CPP库提供了通过端口传递数据流的基本机制。



# Inputs ports

有效输入可以是以下任意一条：

- 可以被节点解析的静态字符串
- 指向Blackboard条目的指针，由条目中的key来指定

blackboard：是树中所有节点共享的key/value存储形式。

entry（条目）：blackboard中的一个key/value对。

输入端口能够从blackboard中读取条目，而输出端口则可以写一个条目。

**假设我们想要创建一个动作节点SaySomething，该节点的回调函数中会打印给定的字符串，我们使用名为message的输入端口来传递这个字符串**

1. 该节点的XML语法为

   ```
       <SaySomething name="first"    message="hello world" />
       <SaySomething name="second"   message="{greetings}" />
   ```

   first node中的message表示传递一个静态字符串“hello world”给端口message，该值在运行时不可以改变;

   second node 则表示读取blackboard中名为greetings的条目的当前值，并赋值给message，这个值在运行时可能会改变。

2. 动作节点SaySomething的实现如下

   ```
   // SyncActionNode (synchronous action) with an input port.
   class SaySomething : public SyncActionNode
   {
     public:
       // If your Node has ports, you must use this constructor signature 
       SaySomething(const std::string& name, const NodeConfiguration& config)
         : SyncActionNode(name, config)
       { }
   
       // It is mandatory to define this static method.定义端口所必需的静态方法
       static PortsList providedPorts()
       {
           // This action has a single input port called "message"
           // Any port must have a name. The type is optional.端口名和端口支持的数据类型
           return { InputPort<std::string>("message") };
       }
   
       // As usual, you must override the virtual function tick()
       NodeStatus tick() override
       {
           // std::optional为C++17语法，它包装了一个特定类型的对象，并用一个额外的bool标志来表示值是否存在。
           Optional<std::string> msg = getInput<std::string>("message");
           // Check if optional is valid. If not, throw its error
           if (!msg)
           {
               throw BT::RuntimeError("missing required input [message]: ", 
                                      msg.error() );
           }
   
           // use the method value() to extract the valid message.
           std::cout << "Robot says: " << msg.value() << std::endl;
           return NodeStatus::SUCCESS;
       }
   };
   ```

3. 用简单函数实现相同功能的节点树，该函数将BT：TreeNode的实例作为输入以访问消息的输入端口，实现如下

   ```
   // Simple function that return a NodeStatus
   BT::NodeStatus SaySomethingSimple(BT::TreeNode& self)
   {
     Optional<std::string> msg = self.getInput<std::string>("message");
     // Check if optional is valid. If not, throw its error
     if (!msg)
     {
       throw BT::RuntimeError("missing required input [message]: ", msg.error());
     }
   
     // use the method value() to extract the valid message.
     std::cout << "Robot says: " << msg.value() << std::endl;
     return NodeStatus::SUCCESS;
   }
   ```

   **注：当一个自定义的树节点拥有输入/输出端口时，这些端口必须在以下静态方法中声明**

   ```
       static MyCustomNode::PortsList providedPorts();
   ```

   使用模板方法TreeNode::getInput<T>(key)便可以从端口message中读取输入值。但是这个方法并不稳定，需要使用者去检测它的返回值的合法性并采取相应的措施：

   - 是否返回了NodeStatus::FAILURE
   - 是否抛出了异常
   - 是否使用了一个不同的默认值

   **建议在tick()函数中调用getInput()方法，而不要在构造函数中调用。C++不会去假定输入的值是静态或者动态的。动态的输入在运行时会发生变化，需要周期性地去读取。**



# Output ports

**只有当另一个节点已经向blackboard的一个条目中写入了某些内容时，一个指向blackboard同一条目的输入端口才是有效的。**

ThinkWhatToSay是一个利用输出端口向条目中写入字符串的例子，实现如下

```
class ThinkWhatToSay : public SyncActionNode
{
  public:
    ThinkWhatToSay(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    static PortsList providedPorts()
    {
        return { OutputPort<std::string>("text") };
    }

    // This Action writes a value into the port "text"
    NodeStatus tick() override
    {
        // the output may change at each tick(). Here we keep it simple.
        setOutput("text", "The answer is 42" );
        return NodeStatus::SUCCESS;
    }
};
```

或者，大多数情况下，出于调试的目的，可以在XML中使用内置操作SetBlackboard将静态值写入条目中

```
 <SetBlackboard   output_key="the_answer" value="The answer is 42" />
```





# 完整的例子

在此示例中，执行了 5 个操作的序列：

- 操作 1 和 4从静态字符串中读取输入message。
- 操作 3 和 5从黑板中名为the_answer的条目中读取输入message。
- 动作 2 在名为the_answer的黑板条目中写入一些东西。

SaySomething2也是一个SimpleActionNode，在注册的时候会看到，它的函数体就是SaySomethingSimple，但是注册名变更为了SaySomething2。

```
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <SaySomething     message="start thinking..." />
           <ThinkWhatToSay   text="{the_answer}"/>
           <SaySomething     message="{the_answer}" />
           <SaySomething2    message="SaySomething2 works too..." />
           <SaySomething2    message="{the_answer}" />
       </Sequence>
    </BehaviorTree>
</root>
```

注册节点的代码如下

```
#include "behaviortree_cpp_v3/bt_factory.h"

// file that contains the custom nodes definitions
#include "dummy_nodes.h"

int main()
{
    using namespace DummyNodes;

    BehaviorTreeFactory factory;

    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

    // SimpleActionNodes can not define their own method providedPorts().
    // We should pass a PortsList explicitly if we want the Action to 
    // be able to use getInput() or setOutput();
    PortsList say_something_ports = { InputPort<std::string>("message") };
    factory.registerSimpleAction("SaySomething2", SaySomethingSimple, 
                                 say_something_ports );

    auto tree = factory.createTreeFromFile("./my_tree.xml");

    tree.tickRoot();

    /*  Expected output:

        Robot says: start thinking...
        Robot says: The answer is 42
        Robot says: SaySomething2 works too...
        Robot says: The answer is 42
    */
    return 0;
}
```

此处，我们利用相同的key（the_answer）将输出端口与输入端口连接起来，即他们指向blackboard的同一个条目；这些端口可以互连也是因为他们的类型相同，都是std::string。
