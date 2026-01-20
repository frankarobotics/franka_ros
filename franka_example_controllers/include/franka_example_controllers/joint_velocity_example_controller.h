// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/StopAction.h>

#include <atomic>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

#include <std_srvs/Trigger.h>

using HomingClient = actionlib::SimpleActionClient<franka_gripper::HomingAction>;
using MoveClient   = actionlib::SimpleActionClient<franka_gripper::MoveAction>;
using GraspClient  = actionlib::SimpleActionClient<franka_gripper::GraspAction>;
using StopClient   = actionlib::SimpleActionClient<franka_gripper::StopAction>;

namespace franka_example_controllers {

enum class GripperState {
  OPEN,
  GRASP,
  HOLD,
  RELEASE
};

class JointVelocityExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  double omega_max{0.2};
  double kp_{3.0};
  ros::Duration elapsed_time_;

  std::unique_ptr<HomingClient> homing_client_;
  std::unique_ptr<MoveClient>   move_client_;
  std::unique_ptr<GraspClient>  grasp_client_;
  std::unique_ptr<StopClient>   stop_client_;

  bool robot_reached_target_;
  ros::Duration stable_time_{0.0};

  const double e_tol_ = 4e-2;    // rad
  const double dq_tol_ = 3e-2;   // rad/s
  const double stable_duration_ = 0.5;  // s

  GripperState gripper_state_;
  bool gripper_cmd_sent_;
  std::atomic<bool> release_requested_{false};//bool release_requested_;
  std::thread input_thread_;
  ros::ServiceServer release_srv_;
  bool released_;

  bool releaseServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
  void triggerRunBarrier();
};

}  // namespace franka_example_controllers
