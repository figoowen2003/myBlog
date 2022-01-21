---
title: Navigation2专题三：URDF补充知识
date: 2021-11-18 14:57:35
tags:
---

# Gazebo模拟传感器

以libgazebo_ros_ray_sensor.so为例，它用于模拟一个激光雷达传感器。

## 1. 在URDF/SDF文件中设置topic

```
	 <sensor name="laser" type="ray">
	   <pose>0.01 0 0.0175 0 -0 0</pose>
           <ray>
             <scan>
               <horizontal>
                 <samples>181</samples>
                 <resolution>1</resolution>
                 <min_angle>-1.57080</min_angle>
                 <max_angle>1.57080</max_angle>
               </horizontal>
             </scan>
             <range>
               <min>0.08</min>
               <max>10</max>
               <resolution>0.05</resolution>
             </range>
           </ray>
           <always_on>1</always_on>
           <update_rate>10</update_rate>
           <visualize>true</visualize>

           <plugin name='laser' filename='libgazebo_ros_ray_sensor.so'>
             <ros>
               <namespace>/demo</namespace>
               <argument>~/out:=scan</argument>
             </ros>
             <output_type>sensor_msgs/LaserScan</output_type>
             <frame_name>laser_link</frame_name>
           </plugin>
	 </sensor>
```

<namespace>/demo</namespace>表示话题的命名空间，<argument>~/out:=scan</argument>表示话题输出的名称为scan，因此话题的全名为/demo/scan

![image-20211118151347923](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118151347923.png)

我们改写一下

```
<plugin name='laser' filename='libgazebo_ros_ray_sensor.so'>
  <ros>
    <namespace>/demo</namespace>
    <argument>--ros-args --remap ~/out:=scan</argument>
  </ros>
  <output_type>sensor_msgs/LaserScan</output_type>
  <frame_name>laser_link</frame_name>
</plugin>
```

重新编译，运行

![image-20211118151634681](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118151634681.png)

增加了--ros-args --remap 参数，表示要将topic重命名为out，此时topic名称为/demo/laser/out

再改写一下

```
<plugin name='laser' filename='libgazebo_ros_ray_sensor.so'>
  <ros>
    <remapping>~/out:=scan</remapping>
  </ros>
  <output_type>sensor_msgs/LaserScan</output_type>
  <frame_name>laser_link</frame_name>
</plugin>
```

![image-20211118154725191](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118154725191.png)

此处取消了namespace的设置，话题名称被重命名为scan

```
<ros>
  <remapping>~/out:=my_scan</remapping>
</ros>
```

![image-20211118155013628](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118155013628.png)

如果不去重命名topic

```
<ros>
  <!--<remapping>~/out:=my_scan</remapping> -->
</ros>
```

![image-20211118161218242](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118161218242.png)

默认话题名称为/laser/out

再改写一下

```
<ros>
  <argument>~/out:=my_scan</argument>
</ros>
```

用argument取代remapping，结果为

![image-20211118161750011](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118161750011.png)

## 2. 在Launch文件中设置topic

```
spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                    arguments=['-entity', 'demo', '0', '0.5', '0.0'],
                    output='screen')
```

![image-20211118162116545](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118162116545.png)

在arguments参数列表中，第二个参数'demo'代表命名空间，因此在孵化机器人模型的过程中会为机器人的参数如topic添加该命名空间，并且此处设置的topic的命名空间优先级高于SDF或者URDF文件中的配置。

需要将第二个参数设置为空

![image-20211118162450214](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118162450214.png)

![image-20211118162624615](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118162624615.png)



# Gazebo与Rviz仿真同步

**问题：将turtlebot3的burger模型与turtlebot3_world世界导入到自定义的孵化器中，在Gazebo中运行该模型，接下来运行turtlebotbot3_navigation，发现无论如何操纵rviz中的2D Pose Estimate去初始化机器人的位置都不成功，自然向Nav2 Goal更是不可能完成。**

![image-20211119161029263](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119161029263.png)

具体操作步骤：

- 从turtlebot3_gazebo源码中拷贝models文件夹中turtlebot3_burger以及turtlebot3_common文件夹到自定义工程的models文件夹下

  ![image-20211119161324923](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119161324923.png)

- 拷贝turtlebot3的世界turtlebot3_world文件夹到自定义工程的models文件夹下

- 修改自定义工程中的.world文件，此处名为warehouse.world

  ![image-20211119161618152](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119161618152.png)

- 修改自定义工程中的launch文件

  ![image-20211119161803619](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119161803619.png)

- 编译运行后，Gazebo正常加载世界与机器人模型，但如果启动nav2却无法加载机器人模型，更无法与Gazebo同步

**原因：**

​	**turtlebot3的模型由SDF格式来完成，rviz无法解析SDF文件，rviz只能解析URDF文件**

**解决方法：**

- 拷贝turtlebot3_gazebo源码models文件夹下的urdf文件夹到自定义工程的models文件夹下

  ![image-20211119162915615](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119162915615.png)

- 修改自定义工程的launch文件，需要启动robot state publisher节点去接卸turtlebot3的urdf文件。拷贝turtlebot3_gazebo源码中的robot_state_publiser.launch.py到自定义工程的launch文件夹下，修改gazebo_world.launch.py文件

  ![image-20211119164344927](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119164344927.png)

- 重新编译，运行，然后在新的terminal中启动nav2（自定义或者turtlebot3的nav2都行）

  ![image-20211119164957893](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119164957893.png)

​		![image-20211119173051632](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119173051632.png)

![image-20211119173156082](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119173156082.png)

![](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/Peek 2021-11-19 17-33.gif)

**注意**

**warehouse.world文件中physics属性的设置会影响机器人的运行速度**

![image-20211119173642564](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211119173642564.png)

其中max_step_size设置过大，机器人会加速运行，而real_time_update_rate设置越大，整体运行速度越慢。
