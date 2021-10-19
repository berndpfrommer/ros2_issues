// -*-c++-*-
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <ros2_issues/msg/test_array_column_major.hpp>
#include <ros2_issues/msg/test_array_column_major2.hpp>
#include <ros2_issues/msg/test_array_complex.hpp>
#include <ros2_issues/msg/test_array_simple.hpp>
#include <thread>
#include <vector>

template <class MsgType>
struct TestPublisher : public rclcpp::Node
{
  explicit TestPublisher(const rclcpp::NodeOptions & options)
  : Node("test_publisher", options)
  {
    pub_ = create_publisher<MsgType>(
      "~/array", declare_parameter<int>("q_size", 1000));
    thread_ = std::thread([this]() {
      rclcpp::Rate rate(declare_parameter<int>("rate", 1000));
      const int numElements = declare_parameter<int>("num_elements", 100);
      rclcpp::Time t_start = now();
      size_t msg_cnt(0);
      const rclcpp::Duration logInterval = rclcpp::Duration::from_seconds(1.0);
      while (rclcpp::ok()) {
        MsgType msg;
        msg.header.stamp = now();
        msg.elements.resize(numElements);
        pub_->publish(msg);
        rate.sleep();
        msg_cnt++;
        rclcpp::Time t = now();
        const rclcpp::Duration dt = t - t_start;
        if (dt > logInterval) {
          RCLCPP_INFO(get_logger(), "pub rate: %8.2f", msg_cnt / dt.seconds());
          t_start = t;
          msg_cnt = 0;
        }
      }
    });
  }
  // -- variables
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
  std::thread thread_;
};

template <class MsgType>
struct TestPublisherColumnMajor : public rclcpp::Node
{
  explicit TestPublisherColumnMajor(const rclcpp::NodeOptions & options)
  : Node("test_publisher", options)
  {
    pub_ = create_publisher<MsgType>(
      "~/array", declare_parameter<int>("q_size", 1000));
    thread_ = std::thread([this]() {
      rclcpp::Rate rate(declare_parameter<int>("rate", 1000));
      const int numElements = declare_parameter<int>("num_elements", 100);
      rclcpp::Time t_start = now();
      size_t msg_cnt(0);
      const rclcpp::Duration logInterval = rclcpp::Duration::from_seconds(1.0);
      MsgType msg;
      msg.x.resize(numElements);
      msg.y.resize(numElements);
      //msg.ts.resize(numElements);
      msg.secs.resize(numElements);
      msg.nsecs.resize(numElements);
      msg.polarity.resize(numElements);

      while (rclcpp::ok()) {
        msg.header.stamp = now();
        pub_->publish(msg);
        rate.sleep();
        msg_cnt++;
        rclcpp::Time t = now();
        const rclcpp::Duration dt = t - t_start;
        if (dt > logInterval) {
          RCLCPP_INFO(get_logger(), "pub rate: %8.2f", msg_cnt / dt.seconds());
          t_start = t;
          msg_cnt = 0;
        }
      }
    });
  }
  // -- variables
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
  std::thread thread_;
};

template <class MsgType>
struct TestPublisherColumnMajor2 : public rclcpp::Node
{
  struct Event
  {
    double ts;
    uint16_t x;
    uint16_t y;
    bool polarity;
  };
  explicit TestPublisherColumnMajor2(const rclcpp::NodeOptions & options)
  : Node("test_publisher", options)
  {
    pub_ = create_publisher<MsgType>(
      "~/array", declare_parameter<int>("q_size", 1000));
    thread_ = std::thread([this]() {
      rclcpp::Rate rate(declare_parameter<int>("rate", 1000));
      const int numElements = declare_parameter<int>("num_elements", 100);
      rclcpp::Time t_start = now();
      size_t msg_cnt(0);
      MsgType msg;
      const rclcpp::Duration logInterval = rclcpp::Duration::from_seconds(1.0);
      while (rclcpp::ok()) {
        // allocate driver data inside loop to simulate driver memory access
        std::vector<Event> driverData(numElements);
        msg.header.stamp = now();
        msg.header.frame_id = "foo";
        msg.x.resize(numElements);
        msg.y.resize(numElements);
        msg.ts.resize(numElements);
        msg.polarity.resize(numElements);
        // copy data from driver (row major) to message (column major)
        for (int i = 0; i < numElements; i++) {
          const auto & e = driverData[i];
          msg.ts[i] =
            (uint64_t)(e.ts * 1e3);  // driver gives t as float in usec
          msg.x[i] = e.x;
          msg.y[i] = e.y;
          msg.polarity[i] = e.polarity;
        }
        pub_->publish(msg);
        rate.sleep();
        msg_cnt++;
        rclcpp::Time t = now();
        const rclcpp::Duration dt = t - t_start;
        if (dt > logInterval) {
          RCLCPP_INFO(get_logger(), "pub rate: %8.2f", msg_cnt / dt.seconds());
          t_start = t;
          msg_cnt = 0;
        }
      }
    });
  }
  // -- variables
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
  std::thread thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if ((argc > 1) && std::string(argv[1]) == "-s") {
    auto node =
      std::make_shared<TestPublisher<ros2_issues::msg::TestArraySimple>>(
        rclcpp::NodeOptions());
    rclcpp::spin(node);
  } else if ((argc > 1) && std::string(argv[1]) == "-c") {
    auto node = std::make_shared<
      TestPublisherColumnMajor<ros2_issues::msg::TestArrayColumnMajor>>(
      rclcpp::NodeOptions());
    rclcpp::spin(node);
  } else if ((argc > 1) && std::string(argv[1]) == "-2") {
    auto node = std::make_shared<
      TestPublisherColumnMajor2<ros2_issues::msg::TestArrayColumnMajor2>>(
      rclcpp::NodeOptions());
    rclcpp::spin(node);
  } else {
    auto node =
      std::make_shared<TestPublisher<ros2_issues::msg::TestArrayComplex>>(
        rclcpp::NodeOptions());
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
