---
title: Navigation2专题九：编写一个新的Costmap2D插件
date: 2021-11-30 10:47:25
tags:
---

# 前置条件

已经安装或编译了ROS2，Gazebo以及Turtlebot3三个功能包。确定Navigation2也在本地进行了编译，可以参考https://navigation.ros.org/build_instructions/index.html#build-instructions。



# 步骤

## 1. 编写Costmap2D插件

作为演示，我们将创建一个costmap插件，这个插件会将重复的代价梯度放到costmap中。代码实例参考navigation2_tutorials仓库的nav2_gradient_costmap_plugin功能包https://github.com/ros-planning/navigation2_tutorials/tree/master/nav2_gradient_costmap_plugin。当然如果想创建自己的Costmap2D图层插件，也可以参考该代码。

- 代码结构如下

  ![image-20211130150315694](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题九：编写一个新的Costmap2D插件/image-20211130150315694.png)

- 头文件gradient_layer.hpp解析

  ```
  #ifndef GRADIENT_LAYER_HPP_
  #define GRADIENT_LAYER_HPP_
  
  #include "rclcpp/rclcpp.hpp"
  #include "nav2_costmap_2d/layer.hpp"
  #include "nav2_costmap_2d/layered_costmap.hpp"
  
  namespace nav2_gradient_costmap_plugin
  {
  
  class GradientLayer : public nav2_costmap_2d::Layer
  {
  public:
    GradientLayer();
  
    virtual void onInitialize();
    virtual void updateBounds(
      double robot_x, double robot_y, double robot_yaw, double * min_x,
      double * min_y,
      double * max_x,
      double * max_y);
    virtual void updateCosts(
      nav2_costmap_2d::Costmap2D & master_grid,
      int min_i, int min_j, int max_i, int max_j);
  
    virtual void reset()
    {
      return;
    }
  
    virtual void onFootprintChanged();
  
    virtual bool isClearable() {return false;}
  
  private:
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  
    // Indicates that the entire gradient should be recalculated next time.
    bool need_recalculation_;
  
    // Size of gradient in cells
    int GRADIENT_SIZE = 20;
    // Step of increasing cost per one cell in gradient
    int GRADIENT_FACTOR = 10;
  };
  
  }  // namespace nav2_gradient_costmap_plugin
  
  #endif  // GRADIENT_LAYER_HPP_
  ```

  插件类nav2_gradient_costmap_plugin::GradientLayer继承自基类nav2_costmap_2d::Layer。

  基类提供了一组虚函数，用于在插件中处理代价地图的图层。这些方法在运行时由LayeredCostmap调用

  | **虚拟方法**         | **方法说明**                                                 | **需要覆盖吗？** |
  | -------------------- | ------------------------------------------------------------ | ---------------- |
  | onInitialize()       | 该方法在插件初始化结束时调用。通常在这个方法中声明 ROS 。任何所需的初始化流程都应该在该方法中完成。 | 不               |
  | updateBounds()       | 调用这个方法会询问插件：需要更新costmap图层的哪个区域。方法有 3 个输入参数：机器人位置和方向；4 个输出参数：指向窗口边界的指针。这些边界用于提升代码性能：只更新窗口内有新的可用信息的区域，避免在每次迭代时更新整个Costmap。 | 是的             |
  | updateCosts()        | 每次需要重新计算costmap时都会调用该方法。它只更新其边界窗口内的成本图层。方法有 4 个输入参数：需要计算的窗口边界； 1 个输出参数：master_grid，表示对resulting costmap的引用。`Layer`类为本插件提供了一个内部的costmap用于完成更新，名为costmap_。如果要利用窗口边界的值对master_grid进行更新，需要使用下面四个方法中的一个：`updateWithAddition()`， `updateWithMax()`，`updateWithOverwrite()`或 `updateWithTrueOverwrite()`。 | 是的             |
  | matchSize()          | 每次更改地图大小时都会调用方法。                             | 不               |
  | onFootprintChanged() | 每次更改足迹时都会调用方法。                                 | 不               |
  | reset()              | 它可能有任何代码要在成本地图重置期间执行。                   | 是的             |

- 源文件gradient_layer.cpp解析

  ```
  #include "nav2_gradient_costmap_plugin/gradient_layer.hpp"
  
  #include "nav2_costmap_2d/costmap_math.hpp"
  #include "nav2_costmap_2d/footprint.hpp"
  #include "rclcpp/parameter_events_filter.hpp"
  
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  using nav2_costmap_2d::NO_INFORMATION;
  
  namespace nav2_gradient_costmap_plugin
  {
  
  GradientLayer::GradientLayer()
  : last_min_x_(-std::numeric_limits<float>::max()),
    last_min_y_(-std::numeric_limits<float>::max()),
    last_max_x_(std::numeric_limits<float>::max()),
    last_max_y_(std::numeric_limits<float>::max())
  {
  }
  
  // This method is called at the end of plugin initialization.
  // It contains ROS parameter(s) declaration and initialization
  // of need_recalculation_ variable.
  void
  GradientLayer::onInitialize()
  {
    auto node = node_.lock(); 
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);
  
    need_recalculation_ = false;
    current_ = true;
  }
  
  // The method is called to ask the plugin: which area of costmap it needs to update.
  // Inside this method window bounds are re-calculated if need_recalculation_ is true
  // and updated independently on its value.
  void
  GradientLayer::updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
    double * min_y, double * max_x, double * max_y)
  {
    if (need_recalculation_) {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      // For some reason when I make these -<double>::max() it does not
      // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
      // -<float>::max() instead.
      *min_x = -std::numeric_limits<float>::max();
      *min_y = -std::numeric_limits<float>::max();
      *max_x = std::numeric_limits<float>::max();
      *max_y = std::numeric_limits<float>::max();
      need_recalculation_ = false;
    } else {
      double tmp_min_x = last_min_x_;
      double tmp_min_y = last_min_y_;
      double tmp_max_x = last_max_x_;
      double tmp_max_y = last_max_y_;
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      *min_x = std::min(tmp_min_x, *min_x);
      *min_y = std::min(tmp_min_y, *min_y);
      *max_x = std::max(tmp_max_x, *max_x);
      *max_y = std::max(tmp_max_y, *max_y);
    }
  }
  
  // The method is called when footprint was changed.
  // Here it just resets need_recalculation_ variable.
  void
  GradientLayer::onFootprintChanged()
  {
    need_recalculation_ = true;
  
    RCLCPP_DEBUG(rclcpp::get_logger(
        "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
      layered_costmap_->getFootprint().size());
  }
  
  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  // Inside this method the costmap gradient is generated and is writing directly
  // to the resulting costmap master_grid without any merging with previous layers.
  void
  GradientLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
    int max_i,
    int max_j)
  {
    if (!enabled_) {
      return;
    }
  
    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!
    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():
    unsigned char * master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
  
    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);
  
    // Simply computing one-by-one cost per each cell
    int gradient_index;
    for (int j = min_j; j < max_j; j++) {
      // Reset gradient_index each time when reaching the end of re-calculated window
      // by OY axis.
      gradient_index = 0;
      for (int i = min_i; i < max_i; i++) {
        int index = master_grid.getIndex(i, j);
        // setting the gradient cost
        unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
        if (gradient_index <= GRADIENT_SIZE) {
          gradient_index++;
        } else {
          gradient_index = 0;
        }
        master_array[index] = cost;
      }
    }
  }
  
  }  // namespace nav2_gradient_costmap_plugin
  
  // This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
  // to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
  // Usually places in the end of cpp-file where the loadable class written.
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
  ```

  1. GradientLayer::onInitialize()包含了ROS参数的声明，并提供了默认值

  ```
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + "." + "enabled", enabled_);
  ```

  2. 同时还设置了need_recalculation_这个边界重计算因子

  ```
  need_recalculation_ = false;
  ```

  3. 如果need_recalculation_为true，则GradientLayer::updateBounds()方法会重新计算窗口边界并更新它们
  4. 在GradientLayer::updateCosts()方法中，梯度直接写入master_grid而不与之前的图层合并。这等同于使用内部的 Layer类提供的内部costmap_，然后调用updateWithTrueOverwrite()方法。下面是主成本地图的梯度算法：

  ```
  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      if (gradient_index <= GRADIENT_SIZE) {
        gradient_index++;
      } else {
        gradient_index = 0;
      }
      master_array[index] = cost;
    }
  }
  ```

  ​	其中：GRADIENT_SIZE是以地图单元格个数表示的每个梯度周期的大小，而GRADIENT_FACTOR表示每一步成本地图值的递减量，如下图所示：

​		![../../_images/gradient_explanation.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题九：编写一个新的Costmap2D插件/gradient_explanation.png)

			5. GradientLayer::onFootprintChanged()方法只是重置 need_recalculation_值。
			5. GradientLayer::reset()方法是虚假的，在本示例插件中没有使用它。该方法保留在这里，是因为需要覆盖父类Layer中的纯虚函数reset()。

## 2. 导出并制作GradientLayer插件

nav2_gradient_costmap_plugin::GradientLayer插件类会被动态加载为nav2_costmap_2d::Layer基类，由LayeredCostmap调用。（C++多态性质）

1. 插件类应该被需要加载的类以插件类基类的形式进行注册，为此，我们使用一个特殊的宏PLUGINLIB_EXPORT_CLASS添加到组成插件库的任何源代码文件中

   ```
   #include "pluginlib/class_list_macros.hpp"
   PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin::GradientLayer, nav2_costmap_2d::Layer)
   ```

   通常这个宏会被放在cpp代码的末尾，当然也可以放在开头

2. 编写插件信息描述文件，此处为gradient_plugins.xml，为xml文件，需要包含以下内容

   ·path：插件所在库的路径和名称。

   ·name：在plugin_types参数中引用的插件类型（有关详细信息，请参阅下一节）。可以是你想要的任何类型。

   ·type：具有命名空间的插件类，该命名空间取自源代码。

   ·basic_class_type：派生插件类的基本父类。

   ·description：文本形式的插件描述。

   ```
   <library path="nav2_gradient_costmap_plugin_core">
     <class name="nav2_gradient_costmap_plugin/GradientLayer" type="nav2_gradient_costmap_plugin::GradientLayer" base_class_type="nav2_costmap_2d::Layer">
       <description>This is an example plugin which puts repeating costs gradients to costmap</description>
     </class>
   </library>
   ```

3. 修改CMakeList.txt

   ```
   <export>
     <costmap_2d plugin="${prefix}/gradient_layer.xml" />
     ...
   </export>
   ```

   通过pluginlib_export_plugin_description_file()这个cmake-function将描述文件包含到CMakeLists.txt中。这个函数会将插件描述文件安装到share目录中，并为插件描述XML设置ament索引以保证该类型的插件可被发现

4. 修改package.xml

   ```
   <export>
     <costmap_2d plugin="${prefix}/gradient_layer.xml" />
     ...
   </export>
   ```

5. 最后编译

   最终的文件结构为

   ![image-20211130160104486](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题九：编写一个新的Costmap2D插件/image-20211130160104486.png)

​		编译命令colcon build --packages-select nav2_gradient_costmap_plugin --symlink-install

## 3. 在Costmap2D中启用插件

1. 将插件告知Costmap2D。

   应将该插件添加到 nav2_params.yaml文件的plugin_names和plugin_types列表中，可以选择放在local_costmap或global_costmap中，以便在运行时为控制器/规划器服务器启用该插件。plugin_names列表包含插件对象的名称，这些名称可以是您想要的任何名称。plugin_types包含plugin_names对象中列出的类型。这些类型应该对应于插件描述XML文件中指定的插件类的name字段。

​		**对于Galactic或更高版本，plugin_names和plugin_types已替换为插件名称的单个plugins字符串		向量。这些类型现在是在 plugin:字段的plugin_name命名空间中定义的如plugin:Myplugin:plugin**

​		![image-20211130160737294](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题九：编写一个新的Costmap2D插件/image-20211130160737294.png)		可能会同时加载多个同一种类型的插件对象。为此， plugin_names列表应包含不同的插件名称，无论plugin_types是否保持相同的类型。例如：

```
plugin_names: ["obstacle_layer", "gradient_layer_1", "gradient_layer_2"] # For Eloquent and earlier
plugin_types: ["nav2_costmap_2d::ObstacleLayer", "nav2_gradient_costmap_plugin/GradientLayer", "nav2_gradient_costmap_plugin/GradientLayer"] # For Eloquent and earlier
plugins: ["obstacle_layer", "gradient_layer_1", "gradient_layer_2"] # For Foxy and later
```

在这种情况下，会通过YAML文件中各个插件对象自己的参数树来处理，如下所示

```
gradient_layer_1:
  plugin: nav2_gradient_costmap_plugin/GradientLayer # For Foxy and later
  enabled: True
  ...
gradient_layer_2:
  plugin: nav2_gradient_costmap_plugin/GradientLayer # For Foxy and later
  enabled: False
  ...
```

## 4. 运行GradientLayer plugin

运行带有已启动Nav2的Turtlebot3的仿真

```
ros2 launch nav2_bringup tb3_simulation_launch.py
```

然后进入到RViz中并单击顶部的“2D Pose Estimate”按钮，并按照“[开始使用Nav2](https://link.zhihu.com/?target=https%3A//navigation.ros.org/getting_started/index.html%23getting-started)”教程所介绍的那样在地图上点击机器人的初始位置。机器人将会在地图上定位，且结果应该如下图所示。这样可以会看到梯度代价地图。还有两个可以注意的事情：代价地图的边界由GradientLayer::updateCosts()动态更新；全局路径由梯度弯曲

![Image of gradient costmap used](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题九：编写一个新的Costmap2D插件/gradient_layer_run.png)
