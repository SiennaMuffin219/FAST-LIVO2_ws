//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef USER_ROS_TOPIC_H_
#define USER_ROS_TOPIC_H_

#include <ros/ros.h>
#include "std_msgs/Time.h"
#include <stdint.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "livox_sdk.h"


namespace livox_ros {

class UserRosTopic {
 public:
  struct time_data {
    uint32_t s;
    uint32_t ns;
  };

  UserRosTopic();
  ~UserRosTopic();

  int Setup();
  int Close();
  int Open(const char *topicname);
  bool IsOpen() { return is_open_; };
  bool HasData() { return has_data; };
  time_data GetData() {
    has_data = false;
    return data;
  };

 private:
  ros::Subscriber timesyncSubscriber;
  ros::NodeHandle nh;
  volatile bool is_open_;
  volatile bool has_data;
  time_data data;

  void LidarTimeSyncCallback(const std_msgs::Time& msg);
};

}  // namespace livox_ros

#endif
