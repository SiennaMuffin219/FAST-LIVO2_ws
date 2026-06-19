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

#include "user_ros_topic.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace livox_ros {

UserRosTopic::UserRosTopic() {
  is_open_ = false;
  has_data = false;
  data = time_data();
  printf("\tUserRosTopic init\n");
}

UserRosTopic::~UserRosTopic() {
  if (is_open_)
    timesyncSubscriber.shutdown();
  is_open_ = false;
  printf("\tUserRosTopic deinit\n");
}

int UserRosTopic::Open(const char *topicname) {
  timesyncSubscriber = nh.subscribe(topicname, 10, &UserRosTopic::LidarTimeSyncCallback, this);
  printf("Subscribed to %s\n", topicname);

  is_open_ = true;
  return 0;
}

int UserRosTopic::Close() {
  if (is_open_)
   timesyncSubscriber.shutdown();
  is_open_ = false;
  return 0;
}

void UserRosTopic::LidarTimeSyncCallback(const std_msgs::Time& msg) {
  data.s = msg.data.sec;
  data.ns = msg.data.nsec;
  has_data = true;
}

}  // namespace livox_ros
