---
title: 基于ROS2设计action之补充一：visibility control
date: 2021-08-23 16:25:29
tags: C++,GCC
---

# 背景

在C语言中，可以使用static关键字，限制函数或者变量的可见性到当前的文件中。共享库包含多个文件，如果需要将某个符号在共享库内的文件间可见，而在库外不可见，隐藏这个符号将变得比较困难。大多数连接器linker都提供了将一个模块的所有符号隐藏或显示的方法，但是如果想要实现符号可选择的显示或隐藏，还需要更多的工作。



# 基本知识

## ELF

ELF：Executable Linkable Format，是UNIX类型操作系统中普遍采用的目标文件格式，文件一般包含代码、数据，有些文件还会包含符号表、调试信息、行号信息、字符串等。

## 目标文件及其类型

目标文件全部是程序的二进制表示，目的是直接在某种处理器上直接执行，如Linux的.so，Windows的DLL。

- 可重定位文件：Relocatable File，包含适合于与其他目标文件链接来创建可执行文件或者共享目标文件的代码和数据，如Linux的*.o，Windows的 *.obj文件。
- 可执行文件：Executable File，包含适合于执行的一个程序，次文件规定了exec()如何创建一个程序的进程映像，如/bin/bash文件， *.exe等
- 共享目标文件：Shared Object File，包含可在两种上下文中链接的代码和数据。首先连接编辑器可以将它和其他可重定位文件和共享目标文件一起处理，生成另外一个目标文件。其次，动态连接器可能将它与某个可执行文件以及其他共享目标一起组合，创建进程映像。



# 使用方法

## 在编译参数中设置

GCC4.0及以上版本提供了-fvisibility的编译选项，可选值为default或者hidden，前者表示编译对象中对于没有显示地设置为隐藏的符号，其属性均对外可见，后者则意味着将隐藏没有进行显示设置的符号。对于编译时没有显示指定的情况，GCC默认为-fvisibility=default。

## 使用attribute机制设置

attribute机制的设置对象包括变量、函数、模板、class等。一般在共享库，尤其是大型的共享库中，编译使用-fvisibility=default用于进行全局符号的隐藏设置，而需要对外展示的符号则用attribute((visibility("default")))进行覆盖性的设置。

```
int a(int n) {return n;}  
   
__attribute__((visibility("hidden"))) int b(int n) {return n;}  
   
__attribute__((visibility("default"))) int c(int n) {return n;}  
   
class X  
{  
    public:  
        virtual ~X();  
};  
   
class __attribute__((visibility("hidden"))) Y  
{  
    public:  
        virtual ~Y();  
};  
   
class __attribute__((visibility("default"))) Z  
{  
    public:  
        virtual ~Z();  
};  
   
X::~X() { }  
Y::~Y() { }  
Z::~Z() { }  
```

含义：

- default：用它定义的符号将被导出，共享库中的函数是默认可见的，意味着该方法对其他模块是可见的。
- hidden：用它定义的符号将不被导出，并且不能从其他对象进行使用，共享库中的函数这是被隐藏的，意味着该方法符号不会被放到动态符号表里，所以其他模块不可以通过该符号表访问该方法。



# 优势

- 从共享库中尽可能少的输出符号是一个好的经验。
- 输出一个受限制的符号会提高程序的模块性，并隐藏实现的细节。
- 共享库装载和识别的符号越少，程序启动和运行的速度就越快。导出所有符号会减慢程序运行速度，并消耗大量内存。



# 测试

**Dynamic_Library/library.hpp**:

```
#ifndef FBC_LIBRARY_LIBRARY_HPP_
#define FBC_LIBRARY_LIBRARY_HPP_
 
// reference: https://gcc.gnu.org/wiki/Visibility
//            https://developer.apple.com/library/content/documentation/DeveloperTools/Conceptual/CppRuntimeEnv/Articles/SymbolVisibility.html
 
#ifdef __GNUC__ >= 4 // it means the compiler is GCC version 4.0 or later
	#ifdef FBC_EXPORT
		#warning "===== dynamic library ====="
		#define FBC_API_PUBLIC __attribute__((visibility ("default")))
		#define FBC_API_LOCAL __attribute__((visibility("hidden")))
	#else
		#warning "===== static library ====="
		#define FBC_API_PUBLIC
		#define FBC_API_LOCAL
	#endif
#else
	#error "##### requires gcc version >= 4.0 #####"
#endif
 
#ifdef __cplusplus
extern "C" {
#endif
 
FBC_API_PUBLIC int library_add(int a, int b);
FBC_API_LOCAL void print_log();
 
#ifdef FBC_EXPORT
FBC_API_PUBLIC int value;
#endif
 
#ifdef __cplusplus
}
#endif
 
template<typename T>
class FBC_API_PUBLIC Simple {
public:
	Simple() = default;
	void Init(T a, T b);
	T Add() const;
 
private:
	T a, b;
};
 
 
#endif // FBC_LIBRARY_LIBRARY_HPP_
```

**Dynamic_Library/library.cpp**:

```
#include "library.hpp"
#include <iostream>
#include <string>
 
FBC_API_PUBLIC int library_add(int a, int b)
{
#ifdef FBC_EXPORT
	value = 11;
#endif
 
	fprintf(stdout, "File: %s, Function: %s, Line: %d\n", __FILE__, __FUNCTION__, __LINE__);
	return (a+b);
}
 
FBC_API_LOCAL void print_log()
{
	fprintf(stderr, "print_log function is hidden, in dynamic library: %s, %d\n", __FUNCTION__, __LINE__);
}
 
template<typename T>
void Simple<T>::Init(T a, T b)
{
	this->a = a;
	this->b = b;
}
 
template<typename T>
T Simple<T>::Add() const
{
	fprintf(stdout, "File: %s, Function: %s, Line: %d\n", __FILE__, __FUNCTION__, __LINE__);
	return (a + b);
}
 
template class Simple<int>;
template class Simple<std::string>;
```

**Test_Dynamic_Library/test_library.hpp**:

```cpp
#ifndef FBC_CPPBASE_TEST_TEST_LIBRARY_HPP_
#define FBC_CPPBASE_TEST_TEST_LIBRARY_HPP_
 
namespace test_library_ {
 
#ifdef __cplusplus
	extern "C" {
#endif
 
int test_library_1();
int test_library_2();
int test_library_3();
 
#ifdef __cplusplus
	}
#endif
 
} // namespace test_library_
 
#endif // FBC_CPPBASE_TEST_TEST_LIBRARY_HPP_
```

**Test_Dynamic_Library/test_library.cpp**:

```cpp
#include "test_library.hpp"
#include <iostream>
#include <string>
 
#include <library.hpp>
 
namespace test_library_ {
 
int test_library_1()
{
	int a{ 4 }, b{ 5 }, c{ 0 };
 
	c = library_add(a, b);
	fprintf(stdout, "%d + %d = %d\n", a, b, c);
 
#ifdef FBC_EXPORT
	fprintf(stdout, "dynamic library: value: %d\n", value);
#endif
 
	return 0;
}
 
int test_library_2()
{
	Simple<int> simple1;
	int a{ 4 }, b{ 5 }, c{ 0 };
 
	simple1.Init(a, b);
	c = simple1.Add();
	fprintf(stdout, "%d + %d = %d\n", a, b, c);
 
	Simple<std::string> simple2;
	std::string str1{ "csdn blog: " }, str2{ "http://blog.csdn.net/fengbingchun" }, str3;
 
	simple2.Init(str1, str2);
	str3 = simple2.Add();
	fprintf(stdout, "contents: %s\n", str3.c_str());
 
	return 0;
}
 
int test_library_3()
{
#ifdef FBC_EXPORT
	fprintf(stdout, "dynamic library cann't run print_log function\n");
#else
	print_log();
#endif
 
	return 0;
}
 
} // namespace test_library_
```

**Test_Dynamic_Library/main.cpp**:

```
#include <iostream>
#include "test_library.hpp"
 
int main()
{
	test_library_::test_library_1();
	test_library_::test_library_2();
	test_library_::test_library_3();
 
	return 0;
}
```

**CMakeList.txt**:

```
# CMake file for Samples_Dynamic_Library

# 设定依赖的CMake版本
CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

# 指定项目名称
PROJECT(Test_Dynamic_Library)

# 打印相关信息, CMAKE_CURRENT_SOURCE_DIR指的是当前处理的CMakeLists.txt所在的路径
MESSAGE(STATUS "current path: ${CMAKE_CURRENT_SOURCE_DIR}")

# 使CMake支持C++11特性
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

# 定义用户自定义变量  
SET(PATH_DYNAMIC_LIBRARY_CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/Dynamic_Library)
SET(PATH_TEST_DYNAMIC_LIBRARY_CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/Test_Dynamic_Library)
SET(PATH_DYNAMIC_LIBRARY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Dynamic_Library)
#MESSAGE(STATUS "Dynamic Library cpp files path: ${PATH_DYNAMIC_LIBRARY_CPP_FILES}")
#MESSAGE(STATUS "Test Dynamic Library cpp files path: ${PATH_TEST_DYNAMIC_LIBRARY_CPP_FILES}")

# 递归查询所有匹配的文件:*.cpp
FILE(GLOB_RECURSE DYNAMIC_LIBRARY_CPP_LIST ${PATH_DYNAMIC_LIBRARY_CPP_FILES}/*.cpp)
FILE(GLOB_RECURSE TEST_DYNAMIC_LIBRARY_CPP_LIST ${PATH_TEST_DYNAMIC_LIBRARY_CPP_FILES}/*.cpp)
#MESSAGE(STATUS "Dynamic Library cpp list: ${DYNAMIC_LIBRARY_CPP_LIST}")
#MESSAGE(STATUS "Test Dynamic Library cpp list: ${TEST_DYNAMIC_LIBRARY_CPP_LIST}")

IF (BUILD_DYNAMIC_LIBRARY)
    # 添加编译参数，添加-D预编译宏定义，可以一次添加多个
    ADD_DEFINITIONS(-DFBC_EXPORT)
    # 生成动态库
    ADD_LIBRARY(Dynamic_Library SHARED ${DYNAMIC_LIBRARY_CPP_LIST})
ELSE()
    ADD_LIBRARY(Static_Library STATIC ${DYNAMIC_LIBRARY_CPP_LIST})
ENDIF()

# 指定需要包含的头文件
INCLUDE_DIRECTORIES(${PATH_DYNAMIC_LIBRARY_INCLUDE_DIR})

# 生成可执行文件Test_Dynamic_Library
ADD_EXECUTABLE(Test_Dynamic_Library ${TEST_DYNAMIC_LIBRARY_CPP_LIST})
IF (BUILD_DYNAMIC_LIBRARY)
    # 用来为target添加需要链接的共享库，指定工程所用的依赖库，包括动态库和静态库
    TARGET_LINK_LIBRARIES(Test_Dynamic_Library Dynamic_Library)
ELSE()
    TARGET_LINK_LIBRARIES(Test_Dynamic_Library Static_Library)
ENDIF()
```

**编译方法**

```
在Linux下通过CMake编译Samples_Dynamic_Library中的测试代码步骤：
将终端定位到Linux_Code_Test/Samples_Dynamic_Library，依次执行如下命令：
$ mkdir build
$ cd build
// Note:-DBUILD_DYNAMIC_LIBRARY=1,编译生成动态库； -DBUILD_DYNAMIC_LIBRARY=0, 编译生成静态库
$ cmake -DBUILD_DYNAMIC_LIBRARY=1 ..
$ make (生成动态库和执行文件)
$ ./Test_Dynamic_Library
```

**参考代码**：https://github.com/fengbingchun/Linux_Code_Test



# ROS2 action实现中的attribute案例

**action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h**

```
#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

//这个分支表示当前系统环境是Windows
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
  	// 表示共享库中的符号可以被导出
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))	
    // 表示共享库中的符号将被隐藏
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else	// 非GNUC时使用declspec
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else	// 表示Linux系统
  #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_TUTORIALS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
```

