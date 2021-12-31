---
title: Navigation2专题二十六：封装遗留代码
date: 2021-12-31 15:46:58
tags:
---

本小节将介绍如何封装一段没有使用BehaviorTree.CPP的代码，原始代码如下

```
// This is my custom type.
struct Point3D { double x,y,z; };

// We want to create an ActionNode to calls method MyLegacyMoveTo::go
class MyLegacyMoveTo
{
public:
    bool go(Point3D goal)
    {
        printf("Going to: %f %f %f\n", goal.x, goal.y, goal.z);
        return true; // true means success in my legacy code
    }
};
```

接下来进行封装

- 首先，需要实现模板方法convertFromString来完成对自定义类型的解析

  ```
  namespace BT
  {
      template <> Point3D convertFromString(StringView key)
      {
          // three real numbers separated by semicolons
          auto parts = BT::splitString(key, ';');
          if (parts.size() != 3)
          {
              throw RuntimeError("invalid input)");
          }
          else{
              Point3D output;
              output.x  = convertFromString<double>(parts[0]);
              output.y  = convertFromString<double>(parts[1]);
              output.z  = convertFromString<double>(parts[2]);
              return output;
          }
      }
  } // end anmespace BT
  ```

- 使用lambda表达式或者std::bind方法封装MyLegacyMoveTo::go方法，并创建一个函数指针，将此函数指针与一个SimpleActionNode绑定。（**如何用std::bind封装？？**）

  ```
  static const char* xml_text = R"(
  
   <root>
       <BehaviorTree>
          <MoveTo  goal="-1;3;0.5" />
       </BehaviorTree>
   </root>
   )";
  
  int main()
  {
      using namespace BT;
  
      MyLegacyMoveTo move_to;
  
      // Here we use a lambda that captures the reference of move_to
      auto MoveToWrapperWithLambda = [&move_to](TreeNode& parent_node) -> NodeStatus
      {
          Point3D goal;
          // thanks to paren_node, you can access easily the input and output ports.
          parent_node.getInput("goal", goal);
  
          bool res = move_to.go( goal );
          // convert bool to NodeStatus
          return res ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
      };
  
      BehaviorTreeFactory factory;
  
      // Register the lambda with BehaviorTreeFactory::registerSimpleAction
  
      PortsList ports = { BT::InputPort<Point3D>("goal") };
      factory.registerSimpleAction("MoveTo", MoveToWrapperWithLambda, ports );
  
      auto tree = factory.createTreeFromText(xml_text);
  
      tree.tickRoot();
  
      return 0;
  }
  
  /* Expected output:
  
  Going to: -1.000000 3.000000 0.500000
  
  */
  ```

  
