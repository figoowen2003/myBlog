---
title: Navigation2专题三：URDF补充知识
date: 2021-11-18 14:57:35
tags:
---

# Gazebo模拟传感器

以libgazebo_ros_ray_sensor.so为例，它用于模拟一个激光雷达传感器。

## 1. 默认的topic

```
```

## 2. 在URDF/SDF文件中设置topic

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

## 3. 在Launch文件中设置topic

```
spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                    arguments=['-entity', 'demo', '0', '0.5', '0.0'],
                    output='screen')
```

![image-20211118162116545](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118162116545.png)

在arguments参数列表中，第二个参数'demo'代表命名空间，因此在孵化机器人模型的过程中会为机器人的参数如topic添加该命名空间，并且此处设置的topicde 命名空间优先级在SDF或者URDF文件之上。

需要将第二个参数设置为空

![image-20211118162450214](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118162450214.png)

![image-20211118162624615](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题三：URDF补充知识/image-20211118162624615.png)

