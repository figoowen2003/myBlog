---
title: ROS2中parameter的设置
date: 2021-08-19 16:51:51
tags: launch, parameter
---

 # 背景

在ROS2中自定义node时，有时候需要增加一些能够被launch文件设置的参数。



# 步骤

## 1. 创建功能包

新建目录dev_ws/src，在该目录下输入以下命令

```
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```

--dependencies参数将必要的依赖添加到package.xml和CMakeList.txt中。

## 2. 编写C++节点

在`dev_ws/src/cpp_parameters/src`目录中，创建一个名为的新文件`cpp_parameters_node.cpp`并将以下代码粘贴到其中：

```
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
      // 创建所需要的参数，参数名为my_parameter，默认值为world
      this->declare_parameter<std::string>("my_parameter", "world");
      // 创建定时器，此处表示1s执行一次respond函数
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
      // 获取参数my_parameter的值，并存入parameter_string_中
      this->get_parameter("my_parameter", parameter_string_);
      RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }
  private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
```

## 3. 修改CMakeList.txt

添加以下内容

```
add_executable(parameter_node src/cpp_parameters_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

install(TARGETS
  parameter_node
  DESTINATION lib/${PROJECT_NAME}
)
```

## 4. 编译运行

```
colcon build --packages-select cpp_parameters
. install/setup.bash
ros2 run cpp_parameters parameter_node
```

![image-20210820145312071](/home/ubuntu-ros2/myBlog/source/_posts/ROS2中parameter的设置/image-20210820145312071.png)

## 5. 修改参数

- 通过命令行来修改

  确保节点正在运行

  ```
  ros2 run cpp_parameters parameter_node
  ```

  在新终端中查看当前的参数列表(会看到自定义参数my_parameter)

  ```
  ros2 param list
  ```

  修改参数为earth

  ```
  ros2 param set /parameter_node my_parameter earth
  ```

  ![image-20210820145745607](/home/ubuntu-ros2/myBlog/source/_posts/ROS2中parameter的设置/image-20210820145745607.png)

- 通过launch文件修改

  在dev_ws/src/cpp_parameters/目录下创建launch目录，新建文件cpp_parameters_launch.py，填写以下内容

  ```
  from launch import LaunchDescription
  from launch_ros.actions import Node
  
  def generate_launch_description():
      return LaunchDescription([
          Node(
              package="cpp_parameters",
              executable="parameter_node",
              name="custom_parameter_node",
              output="screen",
              emulate_tty=True,
              parameters=[
                  {"my_parameter": "earth"}
              ]
          )
      ])
  ```

  在CMakeList中添加

  ```
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  ```

  编译运行省略

  ![image-20210820152150768](/home/ubuntu-ros2/myBlog/source/_posts/ROS2中parameter的设置/image-20210820152150768.png)

- 通过源代码修改（专门细说）
