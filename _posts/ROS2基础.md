---
title: ROS2基础
date: 2021-04-07 11:41:24
tags: ros2,c++,python
---

## 概念

ROS是一个用于在不同进程间匿名的发布、订阅、传递信息的中间件。 ROS2系统的核心部分是ROS网络(ROS Graph)。ROS网络是指在ROS系统中不同的节点间相互通信的连接关系。 ROS Graph这里翻译成了ROS网络，因为我觉得Graph更加抽象，而网络的概念更容易帮助理解其内涵。

##   ROS网络(ROS Graph)概念简介

- ​	节点(Nodes)： 一个节点是一个利用ROS系统和其他节点通信的实体
- ​	消息(Messages)： ROS中在订阅和发布主题时所用到的数据结构
- ​	主题(Topics): 节点可以发布信息到一个主题，同样也可订阅主题来接收消息
- ​	发现(Discovery): 一个自动运行的进程，通过这个进程不同的节点相互发现，建立连接

##   节点(Nodes)

一个节点就是一个在ROS网络中的参与者。 ROS节点通过ROS客户端程序库(ROS client library)来和其他节点进行通信。 节点可以发布或者订阅主题 节点也可以提供ROS服务(Service)。 节点有很多可以配置的相关参数。 节点间的连接时通过一个分布式发现进程来建立的（即上面所说的Discovery）。 不同的节点可以在同一个进程里面，也可以在不同的进程里面，甚至可以在不同的机器上。

## 客户端程序库

ROS客户端程序库可以让不同的语言编写的节点进行通信。 在不同的编程语言中都有对应的ROS客户端程序库(RCL)，这个程序库实现了ROS的基本API。 这样就确保了不同的编程语言的客户端更加容易编写，也保证了其行为更加一致。

下面的客户端程序库是由ROS2团队维护的

- ​	rclcpp = C++ 客户端程序库
- ​	rclpy = Python 客户端程序库

另外其他客户端程序也已经有ROS社区开发出来

## 发现

节点之间的互相发现是通过ROS2底层的中间件实现的。 过程总结如下

1. ​	当一个节点启动后， 它会向其他拥有相同ROS域名(ROS domain， 可以通过设置ROS_DOMAIN_ID环境变量来设置)的节点进行广播，说明它已经上线。 其他节点在收到广播后返回自己的相关信息，这样节点间的连接就可以建立了，之后就可以通信了。
2. ​	节点会定时广播它的信息，这样即使它已经错过了最初的发现过程，它也可以和新上线的节点进行连接。
3. ​	节点在下线前它也会广播其他节点自己要下线了。

节点只会与具有相兼容QOS设置的节点进行通信。

## 例子：发布和接收

在一个终端，启动一个节点(用C++编写)，这个节点会向一个主题发布消息

ros2 run demo_nodes_cpp talker

在另一个终端，同样启动一个节点(用Python编写)，这个节点会订阅和上个节点相同的主题来接收消息。

ros2 run demo_nodes_py listener

你会看到节点自动发现了对方，然后开始互相通信。 你也可以在不同的电脑上启动节点，你也会发现，节点自动建立了它们的连接，然后开始通信。

#   ROS2和不同的DDS程序

ROS2是建立在DDS程序的基础上的。DDS程序被用来发现节点，序列化和传递信息。 [这篇文章](http://design.ros2.org/articles/ros_on_dds.html)(http://design.ros2.org/articles/ros_on_dds.html)详细介绍了DDS程序的开发动机。 总而言之，DDS程序提供了ROS系统所需的一些功能，比如分布式发现节点(并不是像ROS1那样中心化)，控制传输中的不同的"通信质量（Quality of Service）"选项。

DDS是一个被很多公司实现的工业标准。比如RTI的实现[Connext](http://ros2.bwbot.org/tourial/about-ros2/(https:/www.rti.com/products)和eProsima的实现[Fast RTPS](http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps)。 ROS2 支持多种实现方式。因为没有必要“一个尺码的鞋给所有人穿”。用户有选择的自由。 在选择DDS实现的时候你要考虑很多方面：比如法律上你要考虑他们的协议，技术上要考虑是否支持跨平台。 不同的公司也许会为了适应不同的场景提出不止一种的DDS实现方式。 比如RTI为了不同的目标就有很多种他们的Connext的变种。从小到微处理器到需要特殊资质的应用程序（我们支持标准的桌面版）。

为了能够在ROS2中使用一个DDS实现，需要一个ROS中间件(RMW软件包), 这个包需要利用DDS程序提供的API和工具实现ROS中间件的接口。 为了在ROS2中使用一个DDS实现，有大量的工作需要做。但是为了防止ROS2的代码过于绑定某种DDS程序必须支持至少几种DDS程序。因为用户可能会根据他们的项目需求选择不同的DDS程序。

| **名称**                                               | **协议**                                     | **RMW** 				**实现** | **状态**                                                     |
| ------------------------------------------------------ | -------------------------------------------- | -------------------------------- | ------------------------------------------------------------ |
| eProsima *Fast 				RTPS*                   | Apache 				2                     | rmw_fastrtps_cpp                 | 完全支持. 				默认的RMW. 				已经打包在发布的文件中. |
| RTI *Connext*                                          | commercial, 				research         | rmw_connext_cpp                  | 完全支持. 				需要从源码编译支持.                |
| RTI *Connext* (dynamic 				implementation) | commercial, 				research         | rmw_connext_dynamic_cpp          | 停止支持. 				alpha 8.* 之前版本完全支持         |
| PrismTech *Opensplice*                                 | LGPL 				(only v6.4), commercial | rmw_opensplice_cpp               | 停止支持. 				alpha 8.* 之前版本完全支持         |
| OSRF *FreeRTPS*                                        | Apache 				2                     | --                               | 部分支持. 				开发暂停.                          |

### 支持的RMW实现

****暂停支持意味着从 ROS2 alpha 8 版本以后新的添加进ROS2的功能还没有添加到这些中间件的实现中来。 这些中间件的实现也许以后会有也许以后也不会有*




# ROS2客户端简介

客户端程序库是用户在写自己的ROS程序时使用到的ROS API程序。 他们就是用户来访问ROS的基本概念比如节点，主题，服务等等时所使用的程序。 客户端程序库有很多不同语言的实现，这样用户就可以根据他们自己的使用场景灵活的选择最合适的语言。 例如你也许想用Python来写图形程序，因为写起来很方便。但是对于对性能要求比较高的地方，这些程序最好还是用C++来写。

使用不同语言编写的程序之间可以自由的共享信息。因为所有的客户端程序库都提供了代码生成程序，这些程序为用户提供了能够处理ROS的接口文件的能力。

客户端程序除了提供不同语言特定的通信工具之外，还为用户提供了一些使得ROS更加ROS化的核心功能。 比如，这些就是可以通过客户端程序库操作的核心功能

- ​	名称和命名空间
- ​	时间（实际的或者模拟的）
- ​	参数
- ​	终端输出
- ​	线程模型
- ​	跨进程通信

## 支持的客户端程序库

C++ 客户端程序库(rclcpp)和Python客户端程序库(rclpy)都提供了RCL的常用功能。

C++和Python的客户端程序库由 ROS 2 团队维护，ROS2的社区成员同时也支持了以下额外的客户端程序库：

- ​	[JVM 	和 	Android](https://github.com/esteve/ros2_java)
- ​	[Objective 	C 和 	iOS](https://github.com/esteve/ros2_objc)
- ​	[C#](https://github.com/firesurfer/rclcs)
- ​	[Swift](https://github.com/younata/rclSwift)
- ​	[Node.js](https://www.npmjs.com/package/rclnodejs)

## 通用的功能：RCL

大部分客户端程序库提供的功能并不只是在特定的语言里才有。比如参数的效果，命名空间的逻辑，在理想的情况下应该在所有的语言下保持一致。 正是因为这一点，与其为每种语言从零开始实现一遍，倒不如使用一个通用的核心ROS客户端程序库接口。这个接口实现了程序逻辑和ROS中语言无关的功能。这样就使得ROS客户端程序库更小也更容易开发。 由于这个原因，**RCL****通用功能暴露出了****C****的程序接口**。因为C是其他程序语言最容易进行包装的语言。

使用通用的核心库的优点不止可以使得客户端程序更加小型化，同时也使得不同语言的客户端程序库能够保持高度的一致性。 如果通用核心库内的任何功能发生了变化，比如说命名空间，那么所有的客户端程序库的这个功能都会跟着变化。 再者使用通用的核心程序库也意味着当修复bug的时候维护多个客户端程序会更加方便。

[RCL ](http://docs.ros2.org/beta1/api/rcl/)[的 API文档可以看这里](http://docs.ros2.org/beta1/api/rcl/)(https://docs.ros2.org/beta1/api/rcl/)

## 和语言相关的功能

对于和语言相关的功能，并没有在RCL中实现，而是在各个语言的客户端程序库中去实现。 例如，在spin中使用到的线程模型，完全由各语言客户端程序自己实现。

## 与ROS1相比

在ROS1中所有的客户端程序都是从零开始编写的。 这样可以使Python的客户端程序库完全由Python编写。这样就有不用编译源代码的好处。 但是命名规则和其他一些特性并不能和其他客户端程序库保持一致。bug的修复需要在不同的地方重复很多遍。而且有很多功能只有在特定语言的客户端程序库中才实现。

## 总结

通过把ROS通用客户端核心程序库抽出来，不同的语言的客户端程序变得更加容易开发也更能保证功能的一致性

#   ROS 接口

## 1. 背景

ROS程序一般通过一种或两种接口进行通信：消息和服务 ROS使用了一种简化的描述语言来描述这些接口。这种描述语言使得ROS的工具更加容易的自动生成对应语言的源代码。

在这篇文章中，我们将介绍支持的类型和如何创建你的 msg/srv文件

## 2. 消息描述说明

消息的描述文件是在ROS软件包msg文件夹内的.msg文件。 .msg文件包含两个部分：变量域（Fields）和常量(constants)

*这里将**Field翻译成变量域以和constants做作对比区分。如果直接翻译成域，会让人不明所以。*

### 2.1 域

每一个域包含两个部分, 类型和名称。中间用空格隔开，例如

fieldtype1 fieldname1

fieldtype2 fieldname2

fieldtype3 fieldname3

又如

int32 my_int

string my_string

#### 2.1.1 变量域类型

变量域的类型可以是一下几种

- ​	内部定义类型
- ​	有用户自己定义的类型，比如 "geometry_msgs/PoseStamped"

*内部定义的类型现在支持一下几种：* | 类型名称 | [C++](http://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](http://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](http://design.ros2.org/articles/mapping_dds_types.html) | ------------- | ------------- | ----- | ---- | | bool | bool | builtins.bool | boolean | | byte | uint8_t | builtins.bytes *| octet | | char | char | builtins.str* | char | | float32 | float | builtins.float *| float | | float64 | double | builtins.float* | double | | int8 | int8_t | builtins.int *| octet | | uint8 | uint8_t | builtins.int* | octet | | int16 | int16_t | builtins.int *| short | | uint16 | uint16_t | builtins.int* | unsigned short | | int32 | int32_t | builtins.int *| long | | uint32 | uint32_t | builtins.int* | unsigned long | | int64 | int64_t | builtins.int *| long long | | uint64 | uint64_t | builtins.int* | unsigned long long | | string | std::string | builtins.str | string |

*每种内部定义类型都可以用来定义数组* | 类型名称 | [C++](http://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](http://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](http://design.ros2.org/articles/mapping_dds_types.html) | ------------- | ------------- | ----- | ---- | | static array | std::array | builtins.list *| T[N] | | unbounded dynamic array | std::vector | builtins.list | sequence | | bounded dynamic array | custom_class | builtins.list* | sequence | | bounded string | std::string | builtins.str* | string |

所有比ROS变量定义中范围更广，更加宽松的变量，都会被软件限制在ROS所定义的范围中。

*使用数组和限制类型的消息定义的例子*

int32[] unbounded_integer_array

int32[5] five_integers_array

int32[<=5] up_to_five_integers_array



string string_of_unbounded_size

string<=10 up_to_ten_characters_string



string[<=5] up_to_five_unbounded_strings

string<=10[] unbounded_array_of_string_up_to_ten_characters each

string<=10[<=5] up_to_five_strings_up_to_ten_characters_each

#### 2.1.2 变量域名称

变量域名称必须以小写字母开始，同时以下划线作为单词的分割符。不能以下划线结束，也不允许有两个连续的下划线。

#### 2.1.3 变量域默认值

默认值可以设置成变量域类型所允许的任意值。 当前默认值还不能支持字符串数组和复杂类型。（也就是没有出现在内部定义类型里面的，同样也适用于所有的嵌套消息）

定义默认值可以通过在变量域定义中添加第三个元素来实现。也就是

变量域名称 变量域类型 变量域默认值

比如

uint8 x 42

int16 y -2000

string full_name "John Doe"

int32[] samples [-200, -100, 0, 100, 200]

特别说明:

- ​	字符串类型默认值必须用单引号或者双引号括起来
- ​	当前的字符串类型是没有被转义的

### 2.2 常量

常量的定义就好像有默认值的变量域定义。除了常量的值是永远不能由程序改变的。常量通过等号进行赋值。也就是

常量类型 常量名称=常量值

例如

int32 X=123

int32 Y=-123

string FOO="foo"

string EXAMPLE='bar'

特别说明：常量名必须是大写

## 3. 服务定义说明

服务描述由位于ROS包下的srv文件夹内的.srv文件定义。

一个服务描述文件包含了一个请求和一个回应的消息类型。之间用---分割。任意的两个消息类型连接起来，并在中间用---分割都是一个合法的服务描述。

下面是一个非常简单的服务的例子。这个服务接收一个字符串然后返回一个字符串：

string str

\---

string str

当然我们也可以更加复杂一点（如果你想引用来自同一个软件包内的消息类型，那么你一定不要包含这个软件包的名字）：

\#request constants

int8 FOO=1

int8 BAR=2

\#request fields

int8 foobar

another_pkg/AnotherMessage msg

\---

\#response constants

uint32 SECRET=123456

\#response fields

another_pkg/YetAnotherMessage val

CustomMessageDefinedInThisPackage value

uint32 an_integer

你不能在一个服务中嵌入另外一个服务。





 




# QOS（Quality Of Service）简介

ROS2 提供了非常丰富的服务质量控制规则。利用这些规则你可以优化节点间的通信。 合适的服务质量设置可以使得ROS2像TCP协议一样可靠或者像UDP协议一样高效。 在ROS1中，我们只能支持TCP协议。ROS2则受益于底层的DDS传输可以灵活设置。对于在一个容易丢失数据的无线网环境下，可以使用一个高效的服务质量规则。对于在实时计算情况下可以使用高可靠性的服务质量规则。

一组服务质量规则组合在一起就构成了一个服务质量配置文件。 由于在不同的场景中设置合适的服务质量配置文件并不是一个简单的事情。ROS2预先提供了一些常用情景下的配置文件。（比如传感器数据） 同时用户也可以更改服务质量的具体配置。

服务质量文件可以专门用来配置发布者，订阅者，服务提供者和客户端。 一个服务质量文件可以独立应用于不同的上面所说的各种实体。但是如果它们之间使用了不同的服务质量文件那么有可能它们之间会无法建立连接。

## 服务质量规则

一个基本服务质量规则配置文件包含下面的几个规则：

- ​	历史 (History)
  - ​		Keep 	last: 只存储最多N个样本，通过队列深度设置
  - ​		Keep 	all: 存储所有样本，由底层中间件的资源大小限制
- ​	深度 (Depth)
  - ​		队列的大小：只有在和Keep 	last一起时才会起作用
- ​	可靠性 (Reliability)
  - ​		最高效率： 	尝试发送数据，但是在网络不好的情况下有可能丢包
  - ​		高可靠性: 	保证数据发送成功，但是可能会重试发送多次
- ​	耐久力 (Durability)
  - ​		本地缓存(Transient 	local): 发送者会为还未加入的节点保存未接收的数据
  - ​		自动挥发(Volatile): 	不会特意保存数据

对于每个规则，都还有一个系统默认选项。这些值使用了来自底层中间件的参数。默认值可以通过DDS工具进行设置(比如 xml配置文件) DDS本身有着更多的规则可以配置。 由于和ROS1中的功能很相似，所以以上的规则被暴露出来。在未来很有可能会在ROS2中暴露更多的配置规则。

### 和ROS1的对比

历史和深度规则组合起来和ROS1中的队列大小是一样的。

ROS2中的可靠性规则和ROS1中也有对应。ROS1的UDPROS(只在roscpp里面)对应于高效模式。TCPROS(ROS1的默认模式)对应于高可靠性模式。 特别注意即使是ROS2里面的高可靠性模式也是用UDP去实现的。在合适的场景下还可以用来进行广播。

持久性规则和depth=1的规则组合起来和ROS1中的"latching" subscribers很类似。

## QoS 配置文件

配置文件使得开发者可以专注于开发自己的应用程序，不用关心每个服务质量设置。 服务质量配置文件包含了一系列的规则，保证程序能够在特定的使用场景下工作。

目前已经定义的qos配置有

- ​	默认的发布者和订阅者的qos配置

为了能够从ROS1过渡到ROS2，保证网络的相似性是很必要的。 在默认条件下，发布者和订阅者在ROS2中都是配置为**可靠模式****(Reliability)**。

- ​	service

和发布者和订阅者一样服务也是**可靠的**，对于服务来说使用**vilatile**是必须的，因为否则当服务提供者可能会受到已经过时的请求。 尽管这样可以保证客户端不会受到多个应答。服务端却无法保证不受到已经过时的请求。

- ​	传感器数据

对于传感器数据，在大多数情况下及时收到数据更加重要。而保证收到所有的数据并没有这么重要。 也就是开发者希望能尽快的收到最新的数据，尽管可能会有数据丢失。 由于这个原因传感器数据被设置成了**高效模式**和一个较低队列深度。

- ​	参数

ROS2中的参数设置是基于服务的。所以他们有相似的配置。 不同的是参数会有一个更大的队列来保证请求不会丢失。比如当参数客户端无法连接到参数服务器的时候。

- ​	系统默认

使用系统默认的规则

[点击这里](https://github.com/ros2/rmw/blob/release-latest/rmw/include/rmw/qos_profiles.h)(https://github.com/ros2/rmw/blob/release-latest/rmw/include/rmw/qos_profiles.h)可以查看上面的配置使用到的特殊规则。 根据社区的反馈上面的一些规则还需要进一步的调整。

尽管ROS2提供了一些服务质量配置，但是直接配置DDS所提供的规则可以更大程度的发挥出DDS程序的性能。

## 服务质量兼容性

**注意****:** 这一部分只说明了发布者和订阅者的信息，但是这些内容对于服务提供者和服务客户端同样也是适用的。

服务配置文件可以独立配置发布者和订阅者。 只有在发布者和订阅者的服务质量规则是相兼容的时候，它们才能相互建立连接。 服务质量配置文件的兼容性判定是基于“请求对比提供者”模型。只有在请求方，也就是订阅者的规则没有发布者更加严格的条件下才能建立连接。 两者中的更为严格的那个规则会被用于它们之间的这个连接。

在ROS2中暴露的服务质量规则能够影响到连接的是可靠性和耐久性。 下面的表格表示了不同的配置规则和结果：、

*服务质量耐久度兼容性*

| **Publisher**               | **Subscriber**              | **Connection** | **Result**                  |
| --------------------------- | --------------------------- | -------------- | --------------------------- |
| Volatile                    | Volatile                    | Yes            | Volatile                    |
| Volatile                    | Transient 			local | No             | -                           |
| Transient 			local | Volatile                    | Yes            | Volatile                    |
| Transient 			local | Transient 			local | Yes            | Transient 			local |





 





 





 




*服务质量可靠性兼容性*

| **Publisher**           | **Subscriber**          | **Connection** | **Result**              |
| ----------------------- | ----------------------- | -------------- | ----------------------- |
| Best 			effort | Best 			effort | Yes            | Best 			effort |
| Best 			effort | Reliable                | No             | -                       |
| Reliable                | Best 			effort | Yes            | Best 			effort |
| Reliable                | Reliable                | Yes            | Reliable                |

为了保证能够建立一个连接。所有可能影响到兼容性的规则都要注意保证兼容。 也就是即使一个发布者和订阅者有相互兼容的服务质量可靠性规则，但是如果它们的服务质量耐久性规则不兼容，也是无法建立连接的。反之亦然。

# Ros2入们教程

https://www.guyuehome.com/category/column/ros2-tutorials

1. Ubuntu 20.04硬盘多系统安装 https://www.guyuehome.com/8300
2. Ubuntu 20.04安装ROS2 Foxy https://www.guyuehome.com/10226
3. ROS2环境配置 https://www.guyuehome.com/10243  
4. 小海龟仿真器基础使用 https://www.guyuehome.com/10386  
5. 理解节点（Node） https://www.guyuehome.com/10465  
6. 理解话题（Topic） https://www.guyuehome.com/10560  
7. 理解服务（Service） https://www.guyuehome.com/10721  
8. 理解参数（Parameter） https://www.guyuehome.com/10864  
9. 理解动作（Action） https://www.guyuehome.com/10898  
10. rqt_console工具的使用 https://www.guyuehome.com/11162  
11. 创建一个launch文件 https://www.guyuehome.com/11264  
12. 使用ros2 bag录制/回放数据 https://www.guyuehome.com/11320  
13. 创建ros2工作空间 https://www.guyuehome.com/11466  
14. 创建ros2功能包 https://www.guyuehome.com/11507  
15. 创建一个简单的订阅者和发布者（C++） https://www.guyuehome.com/11660  
16. 创建一个简单的订阅者和发布者（python） https://www.guyuehome.com/11701  
17. 创建一个简单的服务器和客户端（C++）https://www.guyuehome.com/12596  

