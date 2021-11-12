---
title: Navigation2专题八：相机校正
date: 2021-11-12 09:00:12
tags:
---

# 概述

本专题展示了如何获取单目相机（monocular camera）的标定参数。



# 要求

1. 安装Camera Calibration Parser、Camera info Manger和Launch Testing Ament Cmake三个功能包，以最新的galactic版本为例

   ```
   sudo apt install ros-<ros2-distro>-camera-calibration-parsers
   ```

   ```
   sudo apt install ros-<ros2-distro>-camera-info-manager
   ```

   ```
   sudo apt install ros-<ros2-distro>-launch-testing-ament-cmake
   ```

   ![image-20211112092540600](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/image-20211112092540600.png)

​	   ![image-20211112092819617](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/image-20211112092819617.png)

​	![image-20211112092929864](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/image-20211112092929864.png)

2. 下载Image Pipeline的源码，然后编译

   ```
   git clone – b <ros2-distro> git@github.com:ros-perception/image_pipeline.git
   ```

   ![image-20211112094349637](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/image-20211112094349637.png)

​		在官网https://github.com/ros-perception/image_pipeline/tree/ros2没有发现源码的对应分支，只好下载了ros2这个分支

​		**编译时只能用colcon build而不可以加上--packages-select image_pipeline**

​		![image-20211112095001385](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/image-20211112095001385.png)

​		**会报一堆的依赖功能包无法找到**

3. 其他材料

   - 一个已知尺寸的大棋盘，本例中使用7X9棋盘格和边长200mm的正方形。由于标定使用棋盘内部的顶点，因此实际使用的是8X10的棋盘。可以从[此处](https://calib.io/pages/camera-calibration-pattern-generator)下载具有设定尺寸的棋盘格。

     ![image-20211112095416083](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/image-20211112095416083.png)

   - 需要位于5mX5m的区域中，没有障碍物和检查版图案。

   - 一个可以通过ROS发布图像的单目相机。



# 步骤

1. 启动单目相机的ROS驱动程序

2. 确保相机通过ROS发布图像

3. 查看是否存在image_raw主题/camera/image_raw

   ```
   ros2 topic list
   ```

   确认它是一个真实的主题并且发布检查话题的hz(频率)

   ```
   ros2 topic hz /camera/image_raw
   ```

   ![../../_images/ROS2_topic_hz.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/ROS2_topic_hz.png)

4. 启动相机标定节点

   ```
   ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.20 image:=/my_camera/image_raw camera:=/my_camera
   ```

   ![image-20211112100514714](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/image-20211112100514714.png)

​		![../../_images/window1.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/window1.png)

6. 为了获得良好的标定结果，需要在相机的视野中移动棋盘格

   - 相机左、右、上、下视野中的棋盘格
     - X 条 - 表示视野中的左/右
     - Y条 - 表示视野中的顶部/底部
     - Size条 - 表示相对于相机是面向接近/远离以及呈倾斜角度
   - 棋盘格填充整个视野
   - 棋盘向左、右、上、下倾斜（Skew条）

   ![../../_images/calibration.jpg](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/calibration.jpg)

7. 随着棋盘在视野中的移动，4个侧边条的长度持续增加。当所有侧边条都呈现绿色并且有足够数据用于标定时，标定按钮亮起。单机标定按钮，一分钟后查看标定结果。

​		![../../_images/greenbars.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/greenbars.png)

8. 标定完成后，保存和提交按钮亮起，同时能在终端中看到结果。

   ![../../_images/calibration_complete.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/calibration_complete.png)

​		![../../_images/calibration_parameters.png](/home/ubuntu-ros2/myBlog/source/_posts/Navigation2专题八：相机校正/calibration_parameters.png)

9. 保存后数据存储到/tmp/calibartiondata.tar.gz。

   ```
   tar -xvf calibration.tar.gz
   ```

   解压标定文件，其中包含可用的标定图形，“ost.yaml”和“ost.txt”文件。yaml文件包含了相机的标定参数。
