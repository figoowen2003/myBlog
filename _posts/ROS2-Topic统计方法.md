---
title: ROS2 Topic统计方法
date: 2021-08-30 17:19:11
tags: topic statics
---

#  背景

ROS2能够为任何的订阅提供消息的统计测量，这有助于用户利用收集到的订阅统计信息判断所使用系统的性能，也可以去做现有问题的诊断

- 如何计算统计信息

  使用libstatistics_collector功能包中的工具在常数时间和常量内存的条件下计算每一个统计集合。每当一个订阅收到新消息时，当前测量窗口都会将这个消息纳入新样本的计算。计算的平均值是一个移动平均值。每收到一个新样本，都会更新最大、最小值以及样本技术，而标准差这是使用Welford Online算法计算。

- 统计信息的类型

  - received message period 接收消息的周期

    单位：毫秒

    使用系统时钟去测量接收消息的周期

  - received message age 接收消息的年龄

    单位：毫秒

    需要在消息头字段添加一个时间戳，以计算从消息发出已经过去了多少时间（年龄）

- 使用方法
  - Topic Statistics方法默认为不开启，需要通过设置订阅者的配置选项来开启指定节点的这一特性。一旦开启，两种类型的统计信息都会启用。
  - 统计信息的消息类型为statistics_msg/msg/MetricsMessage，发送周期默认为1s，对应的话题可以自定义（默认为/statistics）。
  - 接收消息的周期需要在头字段添加消息时间戳，因此可以发布空数据。如果没有找到时间戳，name空数据所有的统计值都是NaN。
  - 每个测量窗口用于周期性统计的第一个采样值都不会产生一个测量方法，因为计算统计值需要知道前一个消息的时间，因此只有一个采样序列才会产生测量方法。



# 案例分析

从github下载demo案例topic_statistics_demo，下载链接：https://github.com/ros2/demos/tree/master/topic_statistics_demo

## 1. 创建消息发布者和使能statistics的消息接收者

```
#include <chrono>
#include <random>
#include <string>

#include "topic_statistics_demo/imu_talker_listener_nodes.hpp"

using namespace std::chrono_literals;

ImuTalker::ImuTalker(
  const std::string & topic_name,
  std::chrono::milliseconds publish_period)
: Node("imu_talker"),
  topic_name_(topic_name),
  publish_period_(publish_period),
  random_generator_(random_number_seed_()),
  random_distribution_(0.0, 1.0) {}

void ImuTalker::initialize()
{
  RCLCPP_INFO(get_logger(), "Talker starting up");

  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    topic_name_,
    10 /* QoS history_depth */);
  publish_timer_ = create_wall_timer(
    publish_period_,
    [this]() -> void {
      publish();
    });
}

void ImuTalker::publish()
{
  sensor_msgs::msg::Imu msg;
  // Timestamp the message to a random time before now,
  // to demonstrate message age metric calculation.

  if (std::floor(random_distribution_(random_generator_) * 2)) {
    RCLCPP_DEBUG(get_logger(), "Adding fixed offset to message timestamp");
    msg.header.stamp =
      this->now() - rclcpp::Duration{0, static_cast<uint32_t>(this->now().nanoseconds() * 0.975)};
  } else {
    msg.header.stamp = this->now();
  }

  RCLCPP_DEBUG(get_logger(), "Publishing header: %u", msg.header.stamp.nanosec);
  publisher_->publish(msg);
}


ImuListener::ImuListener(
  const std::string & topic_name,
  const rclcpp::SubscriptionOptions & subscription_options)
: Node("imu_listener"),
  subscription_options_(subscription_options),	// 此处为订阅者选项，在构造listener时传递进来
  topic_name_(topic_name) {}

void ImuListener::initialize()
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  start_listening();
}

void ImuListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_name_,
      10,  /* QoS history_depth */
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) -> void
      {
        RCLCPP_DEBUG(get_logger(), "Listener heard: %u", msg->header.stamp.nanosec);
      },
      subscription_options_);
  }
}
```

## 2. 创建接收统计消息的订阅者

```
#include <sstream>
#include <string>

#include "topic_statistics_demo/topic_statistics_listener.hpp"

using statistics_msgs::msg::MetricsMessage;	// statistics消息的类型
const char * STATISTIC_TYPES[] = {"unknown", "avg", "min", "max", "std_dev", "sample_count"}; //对应MetricsMessage这一消息类型中的成员

TopicStatisticsListener::TopicStatisticsListener(const std::string & topic_name)
: Node("statistics_listener"),
  topic_name_(topic_name) {}

void TopicStatisticsListener::initialize()
{
  RCLCPP_INFO(get_logger(), "TopicStatisticsListener starting up");
  start_listening();
}

void TopicStatisticsListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<statistics_msgs::msg::MetricsMessage>(
      topic_name_,
      10,  /* QoS history_depth */
      [this](statistics_msgs::msg::MetricsMessage::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(get_logger(), "Statistics heard:\n%s", MetricsMessageToString(*msg).c_str());
      },
      subscription_options_);	// 订阅选项在构造时传递进来
  }
}

std::string TopicStatisticsListener::MetricsMessageToString(const MetricsMessage & results)
{
  std::stringstream ss;
  ss << "Metric name: " << results.metrics_source <<
    " source: " << results.measurement_source_name <<
    " unit: " << results.unit;
  ss << "\nWindow start: " << results.window_start.nanosec << " end: " <<
    results.window_stop.nanosec;

  for (const auto & statistic : results.statistics) {
    ss << "\n" <<
      STATISTIC_TYPES[statistic.data_type] <<
      ": " <<
      std::to_string(statistic.data);
  }

  return ss.str();
}
```

## 3. 显示话题的统计信息

```
  // Configuration variables
  std::string topic_type(argv[1]);
  std::string publish_topic(DEFAULT_PUBLISH_TOPIC);
  std::chrono::milliseconds publish_period(DEFAULT_PUBLISH_PERIOD);
  std::string test_topic("topic_statistics_chatter");

  // Optional argument parsing
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_TOPIC)) {
    publish_topic = rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_TOPIC);
  }
  if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PUBLISH_PERIOD)) {
    publish_period = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, OPTION_PUBLISH_PERIOD)));
  }

  // Initialization and configuration
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // Enable topic statistics publication through subscriptions
  auto options = rclcpp::SubscriptionOptions();	// 创建话题的可选项
  options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;	// 话题统计使能
  options.topic_stats_options.publish_topic = publish_topic;	// 设置消息统计话题的名称，可以自定义，默认为/statistics
  options.topic_stats_options.publish_period = publish_period;	// 设置统计消息发布的周期，默认为5s

  if (topic_type == "imu") {
    // Start the talker and listener nodes to pass IMU messages
    auto talker = std::make_shared<ImuTalker>(test_topic);
    talker->initialize();
    auto listener = std::make_shared<ImuListener>(test_topic, options);	// 创建订阅者，并且开启消息统计
    listener->initialize();

    // Start the listener node to listen to statistics
    auto statistics_listener = std::make_shared<TopicStatisticsListener>(publish_topic);	// 创建统计消息的订阅者，用于显示统计信息
    statistics_listener->initialize();

    // Execution
    executor.add_node(talker);
    executor.add_node(listener);
    executor.add_node(statistics_listener);
    executor.spin();
  }
```

## 4. 编译运行

```
colcon build --packages-select topic_statistics_demo
```

```
ros2 run topic_statistics_demo display_topic_statistics imu
```

![image-20210902154941518](/home/ubuntu-ros2/myBlog/source/_posts/ROS2-Topic统计方法/image-20210902154941518.png)

查看当前的topic

```
ros2 topic list
```

![image-20210902155015642](/home/ubuntu-ros2/myBlog/source/_posts/ROS2-Topic统计方法/image-20210902155015642.png)

使用rqt工具查看整个话题的通信结构

![image-20210902160641357](/home/ubuntu-ros2/myBlog/source/_posts/ROS2-Topic统计方法/image-20210902160641357.png)

也可以使用以下命令查看统计信息的内容

```
ros2 topic echo /statistics
```

![image-20210902160941363](/home/ubuntu-ros2/myBlog/source/_posts/ROS2-Topic统计方法/image-20210902160941363.png)

| data_type value | statistics         |
| --------------- | ------------------ |
| 1               | average            |
| 2               | minimum            |
| 3               | maximum            |
| 4               | standard deviation |
| 5               | sample count       |

