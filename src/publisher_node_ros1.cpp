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

#include <ros/ros.h>
#include <unistd.h>

#include <thread>

#include "ros2_issues/TestArrayComplex.h"
#include "ros2_issues/TestArraySimple.h"

template <class MsgType>
struct TestPublisher
{
  explicit TestPublisher(ros::NodeHandle & nh)
  {
    pub_ = nh.advertise<MsgType>("array", nh.param<int>("q_size", 1000));
    thread_ = std::thread([this, nh]() {
      ros::Rate rate(nh.param<int>("rate", 1000));
      const int numElements = nh.param<int>("num_elements", 100);
      ros::Time t_start = ros::Time::now();
      size_t msg_cnt(0);
      const ros::Duration logInterval(1.0);
      while (ros::ok()) {
        MsgType msg;
        msg.header.stamp = ros::Time::now();
        msg.elements.resize(numElements);
        pub_.publish(msg);
        rate.sleep();
        msg_cnt++;
        ros::Time t = ros::Time::now();
        const ros::Duration dt = t - t_start;
        if (dt > logInterval) {
          ROS_INFO("pub rate: %8.2f", msg_cnt / dt.toSec());
          t_start = t;
          msg_cnt = 0;
        }
      }
    });
  }
  // -- variables
  ros::Publisher pub_;
  std::thread thread_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle pnh("~");
  if ((argc > 1) && std::string(argv[1]) == "-s") {
    auto node =
      std::make_shared<TestPublisher<ros2_issues::TestArraySimple>>(pnh);
    ros::spin();
  } else {
    auto node =
      std::make_shared<TestPublisher<ros2_issues::TestArrayComplex>>(pnh);
    ros::spin();
  }
  ros::shutdown();
  return 0;
}
