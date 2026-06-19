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

#ifndef TIMESYNC_TIMESYNC_H_
#define TIMESYNC_TIMESYNC_H_

#include <thread>
#include "comm_device.h"
#include "comm_protocol.h"
#include "user_uart.h"
#include "user_ros_topic.h"
#include "livox_def.h"

namespace livox_ros {

typedef void (*FnReceiveRmcSyncTimeCb)(const char *rmc, uint32_t rmc_length,
                                    void *client_data);
                                    
typedef void (*FnReceiveUtcSyncTimeCb)(LidarSetUtcSyncTimeRequest* req,
                                    void *client_data);


enum FsmPollState { kOpenDev, kPrepareDev, kCheckDevState, kFsmDevUndef };

typedef struct {
  CommDevConfig dev_config;
  ProtocolConfig protocol_config;
} TimeSyncConfig;

class TimeSync {
 public:
  static TimeSync *GetInstance() {
    static TimeSync time_sync;

    return &time_sync;
  }

  int32_t InitTimeSync(const TimeSyncConfig &config);
  int32_t DeInitTimeSync();
  void StartTimesync() {
    start_poll_state_ = true;
    start_poll_data_ = true;
  }

  int32_t SetReceiveRmcSyncTimeCb(FnReceiveRmcSyncTimeCb cb, void *data) {
    if ((cb != nullptr) || (data != nullptr)) {
      fn_rmc_cb_ = cb;
      client_data_ = data;
      return 0;
    } else {
      return -1;
    }
  }

  int32_t SetReceiveUtcSyncTimeCb(FnReceiveUtcSyncTimeCb cb, void *data) {
    if ((cb != nullptr) || (data != nullptr)) {
      fn_utc_cb_ = cb;
      client_data_ = data;
      return 0;
    } else {
      return -1;
    }
  }

 private:
  TimeSync();
  ~TimeSync();
  TimeSync(const TimeSync &) = delete;
  TimeSync &operator=(const TimeSync &) = delete;

  void PollStateLoop();
  void PollDataLoop();
  void RosSpinLoop();
  void StopTimesync();

  std::shared_ptr<std::thread> t_poll_state_;
  volatile bool exit_poll_state_;
  volatile bool start_poll_state_;

  std::shared_ptr<std::thread> t_poll_data_;
  volatile bool exit_poll_data_;
  volatile bool start_poll_data_;

  std::shared_ptr<std::thread> t_ros_spin_;
  volatile bool exit_ros_spin_;

  TimeSyncConfig config_;
  UserUart *uart_;
  UserRosTopic *ros_topic_;
  CommProtocol *comm_;
  volatile uint32_t rx_bytes_;

  FnReceiveRmcSyncTimeCb fn_rmc_cb_;
  FnReceiveUtcSyncTimeCb fn_utc_cb_;
  void *client_data_;

  volatile uint8_t fsm_state_;
  std::chrono::steady_clock::time_point transfer_time_;
  void FsmTransferState(uint8_t new_state);
  void FsmOpenDev();
  void FsmPrepareDev();
  void FsmCheckDevState();
};

}  // namespace livox_ros
#endif
