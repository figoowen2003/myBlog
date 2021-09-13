---
title: ROS2消息基本知识
date: 2021-09-13 20:00:04
tags:
---

# ROS2通信接口的三种类型

ROS2 应用通常使用messages、services和actions三种类型之一作为通信的接口（communication interfaces）。

ROS2设计了非常简单的描述语言：the interface definition language(IDL)来描述这些接口的类型。

- msg：.msg文件是简单的文本文件，用于描述ROS2中的消息message，他们被用于在不同的变成语言中产生message的源代码。
- srv：.srv文件用于描述ROS2中的service。它由一个request和一个response组成，每个request和response字段都是用message来声明。
- action：.action文件用于描述行为actions。它由一个goal，一个result和feedback三部分组成，每个部分都是用一个message来声明。



# 消息（message）描述规范

## 1. 字段

每个字段由一个类型名和一个变量名组成

```
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```

如：

```
int32 my_int
string my_string
```

### 1.1 字段类型

- 内置类型

- 自定义类型，如“geometry_msgs/PoseStamped”，通常用于消息嵌套

  当前支持的内置类型：

  | 类型名称 | [C++](https://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](https://design.ros2.org/articles/generated_interfaces_python.html) | [DDS类型](https://design.ros2.org/articles/mapping_dds_types.html) |
  | -------- | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | 布尔值   | 布尔值                                                       | 内置的.bool                                                  | 布尔值                                                       |
  | 字节     | uint8_t                                                      | 内建字节数*                                                  | 八位字节                                                     |
  | 字符     | 字符                                                         | 内置的.str*                                                  | 字符                                                         |
  | 浮动32   | 漂浮                                                         | 内置的.float*                                                | 漂浮                                                         |
  | 浮动64   | 双倍的                                                       | 内置的.float*                                                | 双倍的                                                       |
  | 整数8    | int8_t                                                       | 内置函数*                                                    | 八位字节                                                     |
  | uint8    | uint8_t                                                      | 内置函数*                                                    | 八位字节                                                     |
  | 16       | int16_t                                                      | 内置函数*                                                    | 短的                                                         |
  | uint16   | uint16_t                                                     | 内置函数*                                                    | 无符号短                                                     |
  | 整数32   | int32_t                                                      | 内置函数*                                                    | 长                                                           |
  | uint32   | uint32_t                                                     | 内置函数*                                                    | 无符号长                                                     |
  | int64    | int64_t                                                      | 内置函数*                                                    | 长长的                                                       |
  | uint64   | uint64_t                                                     | 内置函数*                                                    | 无符号长长                                                   |
  | 细绳     | 标准::字符串                                                 | 内置的.str                                                   | 细绳                                                         |
  | 字符串   | std::u16string                                               | 内置的.str                                                   | 字符串                                                       |

  内置类型可用于定义数组：

  | 类型名称     | [C++](https://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](https://design.ros2.org/articles/generated_interfaces_python.html) | [DDS类型](https://design.ros2.org/articles/mapping_dds_types.html) |
  | ------------ | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | 静态数组     | std::array<T, N>                                             | 内置列表*                                                    | T[N]                                                         |
  | 无界动态数组 | 标准::向量                                                   | 内置文件列表                                                 | 序列                                                         |
  | 有界动态数组 | custom_class<T, N>                                           | 内置列表*                                                    | 序列<T, N>                                                   |
  | 有界字符串   | 标准::字符串                                                 | 内置的.str*                                                  | 细绳                                                         |

  所有比ROS的定义更宽松的类型，都将使用ROS的约束去限制其范围或长度。

  以下为数组与有界类型的定义示例：

  ```
  int32[] unbounded_integer_array
  int32[5] five_integers_array
  int32[<=5] up_to_five_integers_array
  
  string string_of_unbounded_size
  string<=10 up_to_ten_characters_string
  
  string[<=5] up_to_five_unbounded_strings
  string<=10[] unbounded_array_of_string_up_to_ten_characters_each
  string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
  ```

### 1.2 字段命名

字段名称必须使用小写字母、数字以及下划线。必须以字母开头，不能以下划线结尾，也不能有两个或更多的连续下划线。

### 1.3 设置字段默认值

内置类型以及内置类型数组都可以设置默认值，但当前不支持string数组和复杂类型（非内置类型）的默认值。具体方法为：

```
fieldtype fieldname fielddefaultvalue
```

例如：

```
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```

字符串值需用''或者""，当前字符串值未转义

## 2 常数

ROS2中常数的定义非常类似于给字段赋默认值，但实际上他是常量，不能在程序中修改。具体方法为：

```
constanttype CONSTANTNAME=constantvalue
```

如：

```
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'
```

常量的命名必须为大写字母。



# 服务（service）描述规范

服务文件用"---"来分割请求和响应。

```
string str
---
string str
```

如：

```
#request constants
int8 FOO=1
int8 BAR=2
#request fields
int8 foobar
another_pkg/AnotherMessage msg # 如果使用来自同一个package的消息，则无需写包名
---
#response constants
uint32 SECRET=123456
#response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```



# ROS2 接口类型的新特性

- 有界数组：ROS1的IDL允许无界数组（int32[] foo）和固定大小数组（int32[5] foo），而ROS2的IDL进一步允许有界数组int32[<=5] foo。在某些用例中，更重要的是能够为数组的大小设置上限，而无需承诺始终使用那么多空间。
- 有界字符串：ROS1的IDL允许无界字符串（string foo），而ROS2的IDL允许有界字符串（string<=5 bar）。
- 默认值：ROS1的IDL允许常量字段（int32 X=123），而ROS2则允许指定默认值（int32 X 123），默认值在构造消息或服务时使用，随后还能被覆盖。也可以为action的每个部分指定默认值。
