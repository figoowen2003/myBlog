---
title: Navigation2专题二十八：使用Coroutines的异步Actions
date: 2021-12-31 17:16:37
tags:
---

BehaviorTree.CPP库提供了两种易于使用的抽象类去创建一个异步的动作，这些动作通常具有以下特点：

- 需要很长时间来得出结论
- 可能会返回RUNNING状态
- 能够被终止

第一个抽象类就是我们熟悉的AsyncActionNode，它将在一个独立的线程中执行tick()方法。

在本小结中，我们将引入另一个重要的抽象类CoroActionNode，可以参考https://www.geeksforgeeks.org/coroutines-in-c-cpp/来了解什么是coroutines。

使用Coroutines的主要原因是为了不去孵化一个新的线程以及大幅提高效率，同时也不必担心线程安全问题。

在Coroutines中，开发者需要显示的调用一个yield方法来挂起动作的执行过程。

CoroActionNode封装了这个yield方法，我们可以更方便地使用setStatusRunningAndYield()。



# C++源码Demo

下以下例子可以作为你自己实现CoroActionNode的 "模板"。

```
typedef std::chrono::milliseconds Milliseconds;

class MyAsyncAction: public CoroActionNode
{
  public:
    MyAsyncAction(const std::string& name):
        CoroActionNode(name, {})
    {}

  private:
    // This is the ideal skeleton/template of an async action:
    //  - A request to a remote service provider.
    //  - A loop where we check if the reply has been received.
    //  - You may call setStatusRunningAndYield() to "pause".
    //  - Code to execute after the reply.
    //  - A simple way to handle halt().
    NodeStatus tick() override
    {
        std::cout << name() <<": Started. Send Request to server." << std::endl;

        TimePoint initial_time = Now();
        TimePoint time_before_reply = initial_time + Milliseconds(100);

        int count = 0;
        bool reply_received = false;

        while( !reply_received )
        {
            if( count++ == 0)
            {
                // call this only once
                std::cout << name() <<": Waiting Reply..." << std::endl;
            }
            // pretend that we received a reply
            if( Now() >= time_before_reply )
            {
                reply_received = true;
            }

            if( !reply_received )
            {
                // set status to RUNNING and "pause/sleep"
                // If halt() is called, we will NOT resume execution
                setStatusRunningAndYield();
            }
        }

        // This part of the code is never reached if halt() is invoked,
        // only if reply_received == true;
        std::cout << name() <<": Done. 'Waiting Reply' loop repeated "
                  << count << " times" << std::endl;
        cleanup(false);
        return NodeStatus::SUCCESS;
    }

    // you might want to cleanup differently if it was halted or successful
    void cleanup(bool halted)
    {
        if( halted )
        {
            std::cout << name() <<": cleaning up after an halt()\n" << std::endl;
        }
        else{
            std::cout << name() <<": cleaning up after SUCCESS\n" << std::endl;
        }
    }

    void halt() override
    {
        std::cout << name() <<": Halted." << std::endl;
        cleanup(true);
        // Do not forget to call this at the end.
        CoroActionNode::halt();
    }

    Timepoint Now()
    { 
        return std::chrono::high_resolution_clock::now(); 
    };
};
```

这个动作假装需要等待一个request消息；这个消息将会在100毫秒后到达。

为了让整个过程更有趣，我们创建了一个包含两个动作的序列，但整个序列将在 150 毫秒后因超时而暂停。

```
 <root >
     <BehaviorTree>
        <Timeout msec="150">
            <SequenceStar name="sequence">
                <MyAsyncAction name="action_A"/>
                <MyAsyncAction name="action_B"/>
            </SequenceStar>
        </Timeout>
     </BehaviorTree>
 </root>
```

接下来是main()函数

```
int main()
{
    // Simple tree: a sequence of two asycnhronous actions,
    // but the second will be halted because of the timeout.

    BehaviorTreeFactory factory;
    factory.registerNodeType<MyAsyncAction>("MyAsyncAction");

    auto tree = factory.createTreeFromText(xml_text);

    //---------------------------------------
    // keep executin tick until it returns etiher SUCCESS or FAILURE
    while( tree.tickRoot() == NodeStatus::RUNNING)
    {
        std::this_thread::sleep_for( Milliseconds(10) );
    }
    return 0;
}

/* Expected output:

action_A: Started. Send Request to server.
action_A: Waiting Reply...
action_A: Done. 'Waiting Reply' loop repeated 11 times
action_A: cleaning up after SUCCESS

action_B: Started. Send Request to server.
action_B: Waiting Reply...
action_B: Halted.
action_B: cleaning up after an halt()

*/
```

