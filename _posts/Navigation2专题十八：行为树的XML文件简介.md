---
title: Navigation2专题十八：行为树的XML文件简介
date: 2021-12-28 16:38:35
tags:
---

# XML的基础知识

一棵简单的行为树

```
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SaySomething   name="action_hello" message="Hello"/>
            <OpenGripper    name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>
 </root>
```

- 树的第一个标签<root>，这个标签内可以包含一个或多个子标签<BehaviorTree>
- <root>标签应该包含属性[main_tree_to_execute]
- 如果文件中包含多个子标签<BehaviorTree>，那么属性[main_tree_to_execute]就是必需的，否则可选
- 每个<Behaviortree>标签都应该有属性[ID]
- 每个树节点都由一个单独的标签来描述：
  - 标签的名字就是树节点在工厂中注册的ID
  - 属性[name]是指实例的名称，并且是可选的
  - 端口Ports是使用属性来配置的，如在动作SaySomething中，需要输入端口message
- 子节点的数量
  - 控制节点包含1个到多个子节点
  - 装饰节点和子树只有一个子节点
  - 动作节点和条件节点都是叶子节点



# 端口重映射与指向Blackboards入口的指针

此部分内容需要先学习行为树的教程第二节的知识。

输入/输出端口可以用Blackboard入口的名称来进行重映射，换句话说，就是用BB键值对中的键来重映射。

一个BB的key用语法{key_name}来描述

```
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SaySomething message="Hello"/>
            <SaySomething message="{my_message}"/>
        </Sequence>
     </BehaviorTree>
 </root>
```

在这个例子中

- 序列节点的第一个子节点是“Hello”
- 第二个子节点读取和写入名为“my_message”的BB的key中包含的值



# 简洁的描述 vs 明确的描述

以下两种子节点的写法都是正确的

```
 <SaySomething               name="action_hello" message="Hello World"/>
 <Action ID="SaySomething"   name="action_hello" message="Hello World"/>
```

我们称前一种语法为简洁的，后一种则是明确的

第一个例子如若用明确的语法来描述，则如下所示

```
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
           <Action ID="SaySomething"   name="action_hello" message="Hello"/>
           <Action ID="OpenGripper"    name="open_gripper"/>
           <Action ID="ApproachObject" name="approach_object"/>
           <Action ID="CloseGripper"   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>
 </root>
```

及时简洁的语法更加方便、更容易编写，但是它提供的关于树节点模型的信息太少。类似Groot这样的工具需要明确的语法或者附加的信息。为了兼容Groot，可以使用标签<TreeNodeModel>。

```
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
           <SaySomething   name="action_hello" message="Hello"/>
           <OpenGripper    name="open_gripper"/>
           <ApproachObject name="approach_object"/>
           <CloseGripper   name="close_gripper"/>
        </Sequence>
    </BehaviorTree>

    <!-- the BT executor don't require this, but Groot does -->     
    <TreeNodeModel>
        <Action ID="SaySomething">
            <input_port name="message" type="std::string" />
        </Action>
        <Action ID="OpenGripper"/>
        <Action ID="ApproachObject"/>
        <Action ID="CloseGripper"/>      
    </TreeNodeModel>
 </root>
```

您可以在此处下载[XML 架构](https://www.w3schools.com/xml/schema_intro.asp)： [behaviortree_schema.xsd](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/behaviortree_schema.xsd)。





# 子树

在后续的教程中，我们会尝试在一棵树中包含另一树作为子树，以避免大量的复制粘贴操作，减少复杂度。

在下面的例子中，我们将包含一些动作的行为树“GraspObject”封装到了主行为树中，为简单起见，属性[name]被省略

```
 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence>
           <Action  ID="SaySomething"  message="Hello World"/>
           <SubTree ID="GraspObject"/>
        </Sequence>
     </BehaviorTree>

     <BehaviorTree ID="GraspObject">
        <Sequence>
           <Action ID="OpenGripper"/>
           <Action ID="ApproachObject"/>
           <Action ID="CloseGripper"/>
        </Sequence>
     </BehaviorTree>  
 </root>
```

子树“GraspObject”作为主行为树的节点，将在“SaySomething”之后被执行。



# 包含外部文件

我们也可以使用类似#include的C++方法来导入行为树

```
  <include path="relative_or_absolute_path_to_file">
```

将前面的示例差分为两个文件，每个文件中都有一棵行为树

```
 <!-- file maintree.xml -->

 <root main_tree_to_execute = "MainTree" >

     <include path="grasp.xml"/>

     <BehaviorTree ID="MainTree">
        <Sequence>
           <Action  ID="SaySomething"  message="Hello World"/>
           <SubTree ID="GraspObject"/>
        </Sequence>
     </BehaviorTree>
  </root>
```

```
 <!-- file grasp.xml -->

 <root main_tree_to_execute = "GraspObject" >
     <BehaviorTree ID="GraspObject">
        <Sequence>
           <Action ID="OpenGripper"/>
           <Action ID="ApproachObject"/>
           <Action ID="CloseGripper"/>
        </Sequence>
     </BehaviorTree>  
 </root>
```

**如果要在[ROS 包中](http://wiki.ros.org/Packages)查找文件，可以使用以下语法：**

```
<include ros_pkg="name_package" path="path_relative_to_pkg/grasp.xml"/>
```
