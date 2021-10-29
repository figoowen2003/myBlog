---
title: ROS2开发插件
date: 2021-09-17 11:36:05
tags:
---

# 背景

1. **pluginlib简介**

   pluginlib是用于从ROS功能包中加载和卸载插件的C++库。

   插件是从可以从运行时库（如共享库，动态链接库）加载的类。

2. **功能简介**

   使用pluginlib，我们就不必显示地将应用程序链接到包含所需类的库上，取而代之，pluginlib可以在任何时候打开一个包含需要导出的类的库，而无需提前感知到这个库或者包含所需类的头文件。



# 准备

```
sudo apt-get install ros-rolling-pluginlib
```

安装pluginlib库



# 任务

我们将创建两个功能包，一个用于定义基类，另一个提供插件。基类定义了一个通用的多边形，插件定义指定的形状。

## 1. 创建基类的功能包

我们的工作空间为robot_sim，在src下创建功能包

```
ros2 pkg create --build-type ament_cmake polygon_base --dependencies pluginlib --node-name area_node
```

![image-20210917163357898](/home/ubuntu-ros2/myBlog/source/_posts/ROS2开发插件/image-20210917163357898.png)

![image-20210917163456708](/home/ubuntu-ros2/myBlog/source/_posts/ROS2开发插件/image-20210917163456708.png)

在include/polygon_base目录下新建文件regular_polygon.hpp，写入以下内容

```
#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

// 抽象基类，用于定义通用的多边形
namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP
```

当我们使用pluginlib时，我们的类需要一个不带参数的构造函数，因此，如果需要初始化任何参数，应该使用initialize函数来完成。

为了让这个头文件生效，接下来我们修改CMakeList。

在ament_target_dependencies后添加

```
install(
  DIRECTORY include/
  DESTINATION include
)
```

在ament_package之前添加

```
ament_export_include_directories(
  include
)
```

## 2. 创建插件的功能包

同样，在robot_sim/src下面输入以下命令

```
ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins
```

![image-20210917172640164](/home/ubuntu-ros2/myBlog/source/_posts/ROS2开发插件/image-20210917172640164.png)

接下来，我们将编写抽象基类的两个非虚的实现：

### 2.1 插件的实现

在src下已经存在polygon_plugins.cpp，用以下内容覆盖原有内容

```
#include <polygon_base/regular_polygon.hpp>
#include <cmath>

namespace polygon_plugins
{
  class Square : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        return side_length_ * side_length_;
      }

    protected:
      double side_length_;
  };

  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        return 0.5 * side_length_ * getHeight();
      }

      double getHeight()
      {
        return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
      }

    protected:
      double side_length_;
  };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
```

pluginlib的作用体现在最后三行，通过使用PLUGINLIB_EXPORT_CLASS宏，来完成将类注册为实际插件的过程：

如：

polygon_plugins::Square 插件类的完全限定类型

polygon_base::RegularPolygon 基类的完全限定类型

### 2.2 用来声明插件的XML

pluginlib的宏调用确保了一旦插件所在的库被加载，插件的实例就会被创建。然后，插件的加载器仍然需要一种方法去查找这个库，并且知道去引用库的哪些内容。

因此，我们需要创建一个XML文件，让所有关于插件的必要信息能适用于ROS的工具链

这个xml文件被成为插件描述文件，它存储着关于插件的全部重要信息，如插件所在的库，插件名称，插件类型等等。

新建文件robot/src/polygon_plugins/plugins.xml，写入以下内容

```
<library path="polygon_plugins">
  <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
    <description>This is a square plugin.</description>
  </class>
  <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
    <description>This is a triangle plugin.</description>
  </class>
</library>
```

说明：

1. library标签提供了包含所要加载插件的库的相对路径。在ROS2中，相对路径就是库的名字。在ROS1中，则需要包含前缀lib，有时候还可能是lib/lib，如lib/libpolygon_plugins
2. class标签用于声明从库中导出的插件
   - type参数：插件的完整类型，包括命名空间
   - base_class参数：插件基类的完整类型，包括命名空间
   - description参数：插件功能描述
   - name参数：过去是一个name属性，现在已经不做要求

### 2.3 修改CMakeList

在find_package(pluginlib REQUIRED)之后添加以下内容

```
add_library(polygon_plugins src/polygon_plugins.cpp)
target_include_directories(polygon_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  polygon_plugins
  polygon_base
  pluginlib
)

# plugin description file
pluginlib_export_plugin_description_file(polygon_base plugins.xml)

install(
  TARGETS polygon_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

z在ament_package之前，添加

```
ament_export_libraries(
  polygon_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)
```

## 3. 插件的使用

修改robot_sim/sr/polygon_base/src/area_node.cpp，写入以下内容

```
#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  // ClassLoader是以基类为模板的模板类，其构造函数的第一个参数为基类所属的报名，第二个参数是基类的完整类名
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

  try
  {
    // ClassLoader提供的createSharedInstance函数创建插件的实例
    std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
```

注：基类所在的功能包并不依赖于插件polygon_plugins类。这两个插件将动态地加载到基类功能包中。此处我们对插件名称采用了硬编码方式，也可以使用参数来动态进行。

## 4. 编译运行

在robot_sim下编译

```
colcon build --packages-select polygon_base polygon_plugins
```

![image-20210918164709982](/home/ubuntu-ros2/myBlog/source/_posts/ROS2开发插件/image-20210918164709982.png)

运行

```
. install/setup.bash
```

```
ros2 run polygon_base area_node
```

![image-20210918164828748](/home/ubuntu-ros2/myBlog/source/_posts/ROS2开发插件/image-20210918164828748.png)
