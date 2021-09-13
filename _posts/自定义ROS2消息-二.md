---
title: 自定义ROS2消息(二)
date: 2021-09-13 16:27:18
tags: ROS2 interfaces
---

# 背景

之前已经介绍了如何自定义消息和服务。本文中将进一步介绍如何在一个功能包中定义多个消息、如何在在同一个包中使用自定义消息以及将另一个自定义消息作为字段类型。

在上篇博客中，我们只可以在CMake类型的功能包中定义消息类型，python则只能使用CMake创建的消息。实际上，我们可以将一个消息的定义和Python节点定义在同一个package中（使用ament_cmake_python）。

但本文中为了简单，依然使用CMake与C++作为demo。



# 任务

## 1. 创建功能包more_interfaces

```
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg
```

## 2. 创建msg文件

在more_interfaces/msg下新建文件AddressBook.msg，其内容如下：

```
# 类似于宏定义，FEMALE和MALE代替true和false
bool FEMALE=true
bool MALE=false

string first_name
string last_name
bool gender	# 可以是FEMALE或MALE
uint8 age
string address
```

### 2.1 编译msg文件

在package.xml中添加

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

编译时依赖rosidl_default_generators；运行时则依赖rosidl_default_runtime。

在CMakeList.txt中添加以下几行：

查找从.msg或.srv文件生成消息代码所需要用到的功能包

```
find_package(rosidl_default_generators REQUIRED)
```

声明要生成的消息列表，所有自定的.msg文件都在此处进行设置，确保cmake能够据此重新编译项目

```
set(msg_files
  "msg/AddressBook.msg"
)
```

生成所需要的消息

```
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
```

确保导出消息的运行时依赖

```
ament_export_dependencies(rosidl_default_runtime)
```

这样就完成了编译之前的所有配置。

后续编译由大家自行完成。

### 2.2 在同一个功能包中自定义多个msg/srv

在CMakeList中使用set添加过个消息列表

```
set(msg_files
  "msg/Message1.msg"
  "msg/Message2.msg"
  # etc
  )

set(srv_files
  "srv/Service1.srv"
  "srv/Service2.srv"
   # etc
  )
```

同时也需要生成多个消息的列表

```
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
```

## 3. 使用同一个功能包中的自定义消息

在上一篇文章中，我们谈到，通常会用一个专门的功能包去生成自定义的消息，然后在其他的功能包中去调用这些消息。此处，我们将讲解如何使用同一个功能包下的自定义消息。

### 3.1 编写发布器代码

在more_interfaces/src目录下创建名为publish_address_book.cpp的文件，其内容如下：

```
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"	// include由AddressBook.msg产生的头文件

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
  : Node("address_book_publisher")
  {
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);	// 创建节点和发布器

    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.age = 30;
        message.gender = message.MALE;
        message.address = "unknown";

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };	// 创建回调函数去周期性发布消息
    timer_ = this->create_wall_timer(1s, publish_msg);	// 定时器发布周期为1s
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}
```

### 3.2 编译发布器

在CMakeList中为这个节点创建新的目标

```
# 查找依赖关系
find_package(rclcpp REQUIRED)
# 生成可执行文件
add_executable(publish_address_book
  src/publish_address_book.cpp
)
# 编译时的依赖
ament_target_dependencies(publish_address_book
  "rclcpp"
)
# 目标文件的安装位置
install(TARGETS publish_address_book
 DESTINATION lib/${PROJECT_NAME})
```

为了使用在同一个功能包中定义的消息，我们还需要在CMakeList中加入以下代码，确保目标能够链接到自定义消息上

```
rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
```

这条语句将自动查找由AddressBook.msg所生成的C++代码，并允许目标文件链接到它。

附：**如果目标文件位于其他独立的功能包中，那么rosidl_target_interfaces这一句则是不必要的。**

### 3.3 运行

本文的工作空间都在robot_sim这个文件夹下

```
cd ~/robot_sim
colcon build --packages-up-to more_interfaces
. install/local_setup.bash
ros2 run more_interfaces publish_address_book
```

![image-20210913173104016](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2消息-二/image-20210913173104016.png)

确认消息通过address_book这个话题被发布，新开一个terminal，使用ros2 topic echo命令查看

```
. install/setup.bash
ros2 topic echo /address_book
```

![image-20210913173236525](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2消息-二/image-20210913173236525.png)

## 4. 使用已经存在的自定义消息

假设：已经存在消息类型Contact.msg，它属于功能包rosidl_tutorials_msgs，它的内容与前文中AddressBook.msg完全相同。此时，我们需要定义AddressBook.msg消息类型。

一种方式，就是将Contact.msg的内容拷贝到AddressBook.msg中；

更好的方式为使用Contact去定义AddressBook。

在AddressBook.msg文件中写入以下内容，定义一个Contact类型的数组：

```
rosidl_tutorials_msgs/Contact[] address_book
```

然后修改package.xml文件中的依赖关系：

```
<build_depend>rosidl_tutorials_msgs</build_depend>

<exec_depend>rosidl_tutorials_msgs</exec_depend>
```

接下来修改CMakeList：

```
find_package(rosidl_tutorials_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES rosidl_tutorials_msgs
)
```

如果还存在发布器的cpp文件publish_address_book.cpp，需要在其中添加头文件

```
#include "rosidl_tutorials_msgs/msg/contact.hpp"
```

然后修改回调函数

```
auto publish_msg = [this]() -> void {
   auto msg = std::make_shared<more_interfaces::msg::AddressBook>();
   {
     rosidl_tutorials_msgs::msg::Contact contact;
     contact.first_name = "John";
     contact.last_name = "Doe";
     contact.age = 30;
     contact.gender = contact.MALE;
     contact.address = "unknown";
     msg->address_book.push_back(contact);
   }
   {
     rosidl_tutorials_msgs::msg::Contact contact;
     contact.first_name = "Jane";
     contact.last_name = "Doe";
     contact.age = 20;
     contact.gender = contact.FEMALE;
     contact.address = "unknown";
     msg->address_book.push_back(contact);
   }

   std::cout << "Publishing address book:" << std::endl;
   for (auto contact : msg->address_book) {
     std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
       std::endl;
   }

   address_book_publisher_->publish(*msg);
 };
```

