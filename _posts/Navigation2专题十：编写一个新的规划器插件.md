---
title: Navigation2专题十：编写一个新的规划器插件
date: 2021-12-01 10:02:12
tags:
---

# 概述

如何自定义一个新的规划器插件。

要求在本地机器上已经安装或者构建好了以下功能包：

- ROS2：这里使用galactic版本，可以apt install，也可以本地编译
- Nav2及其依赖包
- Gazebo
- Turtlebot3：可以直接安装或者源码编译



# 实现步骤

## 1. 创建一个新的规划器插件

我们将创建一个简单的直线规划器。教程中的代码也是来自[navigation_tutorials](https://github.com/ros-planning/navigation2_tutorials) 仓库的nav2_straightline_planner功能包。我们也可以参考这个功能包去设计其他的规划器。

本插件类继承自基类nav2_core::GloberPlanner，具体代码结构如下

![image-20211201105018273](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十：编写一个新的规划器插件/image-20211201105018273.png)

- straight_line_planner.hpp解析

  ```
  #ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
  #define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
  
  #include <string>
  #include <memory>
  
  #include "rclcpp/rclcpp.hpp"
  #include "geometry_msgs/msg/point.hpp"
  #include "geometry_msgs/msg/pose_stamped.hpp"
  
  #include "nav2_core/global_planner.hpp"
  #include "nav_msgs/msg/path.hpp"
  #include "nav2_util/robot_utils.hpp"
  #include "nav2_util/lifecycle_node.hpp"
  #include "nav2_costmap_2d/costmap_2d_ros.hpp"
  
  namespace nav2_straightline_planner
  {
  
  class StraightLine : public nav2_core::GlobalPlanner
  {
  public:
    StraightLine() = default;
    ~StraightLine() = default;
  
    // plugin configure
    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  
    // plugin cleanup
    void cleanup() override;
  
    // plugin activate
    void activate() override;
  
    // plugin deactivate
    void deactivate() override;
  
    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override;
  
  private:
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;
  
    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;
  
    // Global Costmap
    nav2_costmap_2d::Costmap2D * costmap_;
  
    // The global frame of the costmap
    std::string global_frame_, name_;
  
    double interpolation_resolution_;
  };
  
  }  // namespace nav2_straightline_planner
  
  #endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
  ```

  基类nav2_core::GlobalPlanner提供了5个纯虚方法来实现规划器插件。

  | **虚拟方法** | **方法说明**                                                 | **需要覆盖吗？** |
  | ------------ | ------------------------------------------------------------ | ---------------- |
  | configure()  | 当规划器服务器进入 on_configure 状态时调用该方法。理想情况下，在该方法中声明 ROS 参数并初始化规划器的成员变量。该方法需要 4 个输入参数：指向父节点的共享指针、规划器名称、tf 缓冲区指针和指向代价地图的共享指针。 | 是的             |
  | activate()   | 当规划器服务器进入 on_activate 状态时调用该方法。理想情况下，此方法应在规划器进入active状态之前实施必要的操作。 | 是的             |
  | deactivate() | 当规划服务器进入 on_deactivate 状态时调用方法。理想情况下，此方法应在规划器进入非活动状态之前实施必要的操作。 | 是的             |
  | cleanup()    | 当规划器服务器进入 on_cleanup 状态时调用方法。理想情况下，此方法应清理已经为规划器创建的各种资源。 | 是的             |
  | createPlan() | 当规划器服务器要求为开始和目标姿势指定全局规划时，将调用该方法。该方法返回nav_msgs::msg::Path类型的数据，该数据携带全局规划信息。该方法需要 2 个输入参数：开始位姿和目标位姿。 | 是的             |

​		本教程中只使用了StraightLine::configure()和StraightLine::createPlan()来实现直线规划器。

- **straight_line_planner.cpp源码解析**

  ```
  #include <cmath>
  #include <string>
  #include <memory>
  #include "nav2_util/node_utils.hpp"
  
  #include "nav2_straightline_planner/straight_line_planner.hpp"
  
  namespace nav2_straightline_planner
  {
  
  void StraightLine::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
  
    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
        0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  }
  
  void StraightLine::cleanup()
  {
    RCLCPP_INFO(
      node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
      name_.c_str());
  }
  
  void StraightLine::activate()
  {
    RCLCPP_INFO(
      node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
      name_.c_str());
  }
  
  void StraightLine::deactivate()
  {
    RCLCPP_INFO(
      node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
      name_.c_str());
  }
  
  nav_msgs::msg::Path StraightLine::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path global_path;
  
    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Planner will only except start position from %s frame",
        global_frame_.c_str());
      return global_path;
    }
  
    if (goal.header.frame_id != global_frame_) {
      RCLCPP_INFO(
        node_->get_logger(), "Planner will only except goal position from %s frame",
        global_frame_.c_str());
      return global_path;
    }
  
    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;
    // calculating the number of loops for current value of interpolation_resolution_
    int total_number_of_loop = std::hypot(
      goal.pose.position.x - start.pose.position.x,
      goal.pose.position.y - start.pose.position.y) /
      interpolation_resolution_;
    double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;
  
    for (int i = 0; i < total_number_of_loop; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = start.pose.position.x + x_increment * i;
      pose.pose.position.y = start.pose.position.y + y_increment * i;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  
    global_path.poses.push_back(goal);
  
    return global_path;
  }
  
  }  // namespace nav2_straightline_planner
  
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
  ```

  其中，configure()方法必须用来设置ROS参数和进行必要的初始化操作：

  name_+".interpolation_resolution"是获取规划器指定的interpolation_resolution参数。在Navigation2中，允许加载多个插件并保证器组织有序，每个插件都会映射到某个ID或名称上。如果想要检索某个插件的参数，可以使用<mapped_name_of_plugin>.<name_of_parameter>，比如node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_); 本教程中，规划器映射到名称“GridBased”，检索其中的interpolation_resolution参数，需要使用GridBased.interpolation_resolution。

  在createPlan()方法中，需要创建从起始位姿到目标位姿的一条路径，成功后路径会被转换为nav_msgs::msg::Path会返回给规划器服务器。

- **导出规划器插件**

  本教程中，nav2_straightline_planner::StraightLine 类将被动态加载为nav2_core::GlobalPlanner基类。

  1. 修改cpp源码

     使用了pluginlib来导出插件类，它提供宏PLUGINLIB_EXPORT_CLASS来完成导出工作。

     ```
     #include "pluginlib/class_list_macros.hpp"
     PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
     ```

     通常将这两行放到文件末尾。

  2. 创建插件描述文件global_planner_plugin.xml，需要包含以下内容

     ·library path：插件库名称及其位置。

     ·class name：类的名称。

     ·class type：类的类型。

     ·base class：基类的名称。

     ·description：插件的描述。

     ```
     <library path="nav2_straightline_planner_plugin">
       <class name="nav2_straightline_planner/StraightLine" type="nav2_straightline_planner::StraightLine" base_class_type="nav2_core::GlobalPlanner">
         <description>This is an example plugin which produces straight path.</description>
       </class>
     </library>
     ```

  3. 修改CMakeList.txt

     ```
     pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
     ```

     使用cmake函数pluginlib_export_plugin_desription_file()将描述文件安装到share目录中，并设置ament索引以使其可被发现。

  4. 修改package.xml

     ````
     <export>
       <build_type>ament_cmake</build_type>
       <nav2_core plugin="${prefix}/global_planner_plugin.xml" />
     </export>
     ````

  5. 编译。

- 通过params文件传递插件名称

  需要修改nav2_params.yaml文件，将以下参数

  ```
  planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_navfn_planner/NavfnPlanner"] # For Eloquent and earlier
    planner_plugin_ids: ["GridBased"] # For Eloquent and earlier
    plugins: ["GridBased"] # For Foxy and later
    use_sim_time: True
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner # For Foxy and later
      tolerance: 2.0
      use_astar: false
      allow_unknown: true
  ```

  替换为

  ```
  planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_straightline_planner/StraightLine"] # For Eloquent and earlier
    planner_plugin_ids: ["GridBased"] # For Eloquent and earlier
    plugins: ["GridBased"] # For Foxy and later
    use_sim_time: True
    GridBased:
      plugin: nav2_straightline_planner/StraightLine # For Foxy and later
      interpolation_resolution: 0.1
  ```

  对galactic及以上的版本，plugin_names和plugin_types会被替换为单个plugin的字符串向量。types当前是被定义在plugin:这个字段的plugin_name命名空间中，如plugin:MyPlugin::Plugin。

- 运行StraightLine插件

  ```
  ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params_file.yaml
  ```

  然后进入到RViz中并单击顶部的“2D Pose Estimate”按钮，并按照“[开始使用Nav2](https://link.zhihu.com/?target=https%3A//navigation.ros.org/getting_started/index.html%23getting-started)”教程所介绍的那样在地图上点击机器人的初始位置。机器人将会在地图上定位，然后点击“Navigation2 goal”按钮，并在您想要规划器将其视作目标位姿的位置上单击。这样，规划器就会规划该路径，且机器人就会开始朝着目标移动。

​		![Animated gif with gradient demo](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题十：编写一个新的规划器插件/nav2_straightline_gif.gif)
