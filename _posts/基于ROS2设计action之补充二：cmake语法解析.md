---
title: 基于ROS2设计action之补充二：cmake语法解析
date: 2021-08-24 11:40:32
tags:
---

# cmake之add_library

**语法**

```
add_library(<name> [STATIC | SHARED | MODULE]
            [EXCLUDE_FROM_ALL]
            source1 [source2 ...])
```

name：库的名字，无需加lib，会自动添加这个前缀

STATIC | SHARED | MODULE：静态库 | 动态库 | 在使用dyld的系统有效，否则被视为SHARED

EXCLUDE_FROM_ALL：这个库不会被默认构建，除非有其他的组件依赖或者手动构建

**使用**

```
SET(LIBHELLO_SRC hello.c)
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})
ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})
```

**注意**

上面构建的libhello.so与libhello_static.a，本质区别一个是动态库，一个是静态库，合理的命名为libhello.so与libhell.a，但显然名字不同。原因在于，如果将第三行代码改成

```
ADD_LIBRARY(hello STATIC ${LIBHELLO_SRC})
```

则会因为重名，导致静态库无法构建。

解决办法为，修改libhello_static.a的属性-OUTPUT_NAME

```
SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME "hello")
```

这样就可以生成libhello.so与libhello.a了



# cmake之target_include_directories

**语法**

```
target_include_directories(<target> [SYSTEM] [BEFORE]
  <INTERFACE|PUBLIC|PRIVATE> [items1...]
  [<INTERFACE|PUBLIC|PRIVATE> [items2...] ...])
```

指定编译target时需要包含的目录或者目标。

target必须是由add_executable()或者add_library()命令创建的目标

INTERFACE|PUBLIC|PRIVATE关键字用于指定items1等参数的范围，PRIVATE和PUBLIC的items将产生target的INCLUDE_DIRECTORIES属性，PUBLIC和INTERFACE的items将产生target的INTERFACE_INCLUDE_DIRECTORIES属性，也就是添加了公共默认搜索路径。。

target_include_directories的参数也可以使用“生成表达式”$<...>这样的语法，如$<BUILD_INTERFACE>和$<INSTALL_INTERFACE>被用于描述不同的使用要求。

```
target_include_directories(mylib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mylib>
  $<INSTALL_INTERFACE:include/mylib>  # <prefix>/include/mylib
)
```



# cmake之target_compile_definitions

**语法**

```
target_compile_definitions(<target>
   <INTERFACE|PUBLIC|PRIVATE> [items1...]
   [<INTERFACE|PUBLIC|PRIVATE> [items2...] ...])
```

给target添加编译选项，格式与target_include_directories类似。

PRIVATE 和 PUBLIC 项将产生 target>的 COMPILE_DEFINITIONS 属性。PUBLIC 和 INTERFACE 项将产生 target>的INTERFACE_COMPILE_DEFINITIONS 属性。其后的参数指定编译定义。重复调用相同的目标将按照调用顺序追加（定义）。

target_compile_definitions的参数可以使用带语法$<...>的“生成表达式”。

```reasonml
target_compile_definitions(foo PUBLIC FOO)
target_compile_definitions(foo PUBLIC -DFOO)  # -D removed
target_compile_definitions(foo PUBLIC "" FOO) # "" ignored
target_compile_definitions(foo PUBLIC -D FOO) # -D becomes "", then ignored

#定义带值的
target_compile_definitions( Tutorial PRIVATE "LOG_LEVEL=2" )
```



# cmake+ROS2之rclcpp_components_register_node

**背景**

ROS2中的组件，类似与ROS1中的nodelet，这样无需写main函数，而是直接将组件编译成库。

组件通常都是rclcpp::Node的子类，因为组件不能控制线程，所以其构造函数不能运行占用太多时间或者阻塞的任务。

**语法**

写好一个Node，然后加上下面那句

```
#include "rclcpp_components/register_node_macro.hpp"
 
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Talker)
```

然后组件的CMakeLists.txt中必须加上

```
add_library(talker_component SHARED
   src/talker_component.cpp)
rclcpp_components_register_nodes(talker_component "composition::Talker")
# To register multiple components in the same shared library, use multiple calls
# rclcpp_components_register_nodes(talker_component "composition::Talker2")
```



# action设计中的示例

```
cmake_minimum_required(VERSION 3.8)
project(action_tutorials_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# 表示将cpp编译为名为action_server的共享库
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
# 编译action_server所依赖的头文件的搜索路径
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 在编译时灵活注入需要的宏定义
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

