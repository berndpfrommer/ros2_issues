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


#include <chrono>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <ros2_issues/msg/test_array_column_major.hpp>
#include <ros2_issues/msg/test_array_complex.hpp>
#include <ros2_issues/msg/test_array_simple.hpp>
#include <thread>

using namespace std::chrono_literals;

template <class MsgType>
struct TestSubscriber : public rclcpp::Node
{
  explicit TestSubscriber(const rclcpp::NodeOptions & options)
  : Node("test_subscriber", options)
  {
    window_start_time = get_clock()->now();

    auto callback =
      [this](typename MsgType::ConstSharedPtr msg) -> void {
        (void)msg;
        this->window_num_msg++;
      };
    sub_ = create_subscription<MsgType>(
      "/test_publisher/array",
      100,
      callback);

    stats_timer_ = create_wall_timer(1s, [this]() {
      const rclcpp::Time t(this->get_clock()->now());
      const double elapsed = (t - window_start_time).seconds();
      const double msg_per_sec = this->window_num_msg / elapsed;

      RCLCPP_INFO(
          get_logger(),
          "rate: %.6f msgs/sec   messages received: %d   elapsed seconds: %.6f",
          msg_per_sec,
          window_num_msg,
          elapsed);
      this->window_num_msg = 0;
      window_start_time = t;
    });
  }

  // -- variables
  int window_num_msg = 0;
  rclcpp::Time window_start_time;
  typename rclcpp::Subscription<MsgType>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if ((argc > 1) && std::string(argv[1]) == "-s") {
    auto node =
      std::make_shared<TestSubscriber<ros2_issues::msg::TestArraySimple>>(
        rclcpp::NodeOptions());
    rclcpp::spin(node);
  } 
  else if ((argc > 1) && std::string(argv[1]) == "-c") {
    auto node =
      std::make_shared<TestSubscriber<ros2_issues::msg::TestArrayColumnMajor>>(
        rclcpp::NodeOptions());
    rclcpp::spin(node);
  } else {
    auto node =
      std::make_shared<TestSubscriber<ros2_issues::msg::TestArrayComplex>>(
        rclcpp::NodeOptions());
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
