// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/trigger_rate.h>

namespace franka_hw {

TriggerRate::TriggerRate(double rate) : period_(1.0 / rate), time_stamp_(ros::Time::now()) {}

bool TriggerRate::operator()() {
  auto now = ros::Time::now();
  if (now - time_stamp_ >= period_ || now < time_stamp_) {
    time_stamp_ = now;
    return true;
  }
  return false;
}

}  // namespace franka_hw
