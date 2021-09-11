---
title: 自定义ROS2 msg与srv文件
date: 2021-09-11 11:09:19
tags: msg, srv
---

# 背景

使用预定义的消息和服务是一种很好的实践，但有些场景下也需要用户去定义自己的消息和服务类型。本文将介绍创建自定义消息和服务的基本方法。



# 任务

## 1. 新建功能包cpp_tutorial_interfaces

我们将在这个功能包中定义所需要的.msg和.srv文件，目前的做法都是将自定义消息和服务类型放到独立的功能包中，然后在同一个工作空间下用另一个功能包去使用他们。

在robot_sim/src目录下输入以下命令

```
ros2 pkg create --build-type ament_cmake cpp_tutorial_interfaces
```

在当前的ROS2版本中（rolling），只有C++的功能包可以自定义消息和服务，也就是说在建包的时候需要选择ament_cmake方式，但是python的功能包可以导入由C++功能包创建的自定义消息和服务。

![image-20210911174159435](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2-msg与srv文件/image-20210911174159435.png)

在cpp_tutorial_interfaces目录下新建msg和srv目录。

## 2. 自定义消息和服务

在cpp_tutorial_interfaces/msg目录下新建文件Num.msg，在该文件中写入

```
int64 num
```

在cpp_tutorial_interfaces/srv目录下新建文件AddThreeInts.srv，在该文件中写入

```
int64 a
int64 b
int64 c
---
int64 sum
```

## 3. 修改CMakeList

在CMakeList.txt中添加

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )
```

## 4. 修改package.xml

在xml文件中写入

```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 5. 编译

```
colcon build --packages-select cpp_tutorial_interfaces
```

## 6. 验证

```
. install/setup.bash
```

使用ros2 interface show命令

```
ros2 interface show cpp_tutorial_interfaces/msg/Num
```

![image-20210911174951658](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2-msg与srv文件/image-20210911174951658.png)

```
ros2 interface show cpp_tutorial_interfaces/srv/AddThreeInts
```

![image-20210911175053222](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2-msg与srv文件/image-20210911175053222.png)

## 7. 测试Num.msg

新建功能包cpp_custom_msgs

```
ros2 pkg create --build-type ament_cmake cpp_custom_msgs
```

发布者minimal_publisher.cpp

```
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cpp_tutorial_interfaces/msg/num.hpp"     // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<cpp_tutorial_interfaces::msg::Num>("topic", 10);    // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = cpp_tutorial_interfaces::msg::Num();                               // CHANGE
    message.num = this->count_++;                                        // CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.num);    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<cpp_tutorial_interfaces::msg::Num>::SharedPtr publisher_;         // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

接收者minimal_subscriber.cpp

```
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cpp_tutorial_interfaces/msg/num.hpp"     // CHANGE
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<cpp_tutorial_interfaces::msg::Num>(          // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const cpp_tutorial_interfaces::msg::Num::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg->num);              // CHANGE
  }
  rclcpp::Subscription<cpp_tutorial_interfaces::msg::Num>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

CMakeList

```
cmake_minimum_required(VERSION 3.8)
project(cpp_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cpp_tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/minimal_publisher.cpp)
ament_target_dependencies(talker rclcpp cpp_tutorial_interfaces)         # CHANGE

add_executable(listener src/minimal_subscriber.cpp)
ament_target_dependencies(listener rclcpp cpp_tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml

```
cmake_minimum_required(VERSION 3.8)
project(cpp_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cpp_tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/minimal_publisher.cpp)
ament_target_dependencies(talker rclcpp cpp_tutorial_interfaces)         # CHANGE

add_executable(listener src/minimal_subscriber.cpp)
ament_target_dependencies(listener rclcpp cpp_tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

编译

```
colcon build --packages-select cpp_custom_msgs
```

运行

```
. install/setup.bash
ros2 run cpp_custom_msgs talker
```

![image-20210911175822990](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2-msg与srv文件/image-20210911175822990.png)

```
. install/setup.bash
ros2 run cpp_custom_msgs listener
```

![image-20210911175856453](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2-msg与srv文件/image-20210911175856453.png)

## 8. 测试AddThreeInts.srv

新建功能包cpp_custom_srv

```
ros2 pkg create --build-type ament_cmake cpp_custom_srv
```

服务端add_three_ints_server.cpp

```
#include "rclcpp/rclcpp.hpp"
#include "cpp_tutorial_interfaces/srv/add_three_ints.hpp"     // CHANGE

#include <memory>

void add(const std::shared_ptr<cpp_tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
          std::shared_ptr<cpp_tutorial_interfaces::srv::AddThreeInts::Response>       response)  // CHANGE
{
  response->sum = request->a + request->b + request->c;                                       // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",   // CHANGE
                request->a, request->b, request->c);                                          // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");  // CHANGE

  rclcpp::Service<cpp_tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =                 // CHANGE
    node->create_service<cpp_tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);     // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");      // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

客户端add_three_ints_client.cpp

```
#include "rclcpp/rclcpp.hpp"
#include "cpp_tutorial_interfaces/srv/add_three_ints.hpp"        // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");      // CHANGE
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client"); // CHANGE
  rclcpp::Client<cpp_tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                        // CHANGE
    node->create_client<cpp_tutorial_interfaces::srv::AddThreeInts>("add_three_ints");                  // CHANGE

  auto request = std::make_shared<cpp_tutorial_interfaces::srv::AddThreeInts::Request>();               // CHANGE
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);               // CHANGE

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

// 这一段的rclcpp::executor::FutureReturnCode只支持foxy版本
//   auto result = client->async_send_request(request);
//   // Wait for the result.
//   if (rclcpp::spin_until_future_complete(node, result) ==
//     rclcpp::executor::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
//   } else {
//     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
//   }

//   rclcpp::shutdown();

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}
```

CMakeList

```
cmake_minimum_required(VERSION 3.8)
project(cpp_custom_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cpp_tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_three_ints_server.cpp)
ament_target_dependencies(server
  rclcpp cpp_tutorial_interfaces)                      #CHANGE

add_executable(client src/add_three_ints_client.cpp)
ament_target_dependencies(client
  rclcpp cpp_tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cpp_custom_srv</name>
  <version>0.0.0</version>
  <description>custom srv cs</description>
  <maintainer email="figoowen2003@126.com">ubuntu-ros2</maintainer>
  <license>Apache 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>cpp_tutorial_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

编译

```
colcon build --packages-select cpp_custom_srv
```

运行

```
. install/setup.bash
ros2 run cpp_custom_srv server
```

![image-20210911180415407](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2-msg与srv文件/image-20210911180415407.png)

```
. install/setup.bash
ros2 run cpp_custom_srv client
```

![image-20210911180516835](/home/ubuntu-ros2/myBlog/source/_posts/自定义ROS2-msg与srv文件/README.md)
