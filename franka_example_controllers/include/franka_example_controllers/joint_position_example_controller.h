// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/StopAction.h>

#include <std_srvs/Trigger.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Bool.h>  
#include <ros/node_handle.h>
#include <ros/time.h>

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

class JointPositionExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  std::unique_ptr<HomingClient> homing_client_;
  std::unique_ptr<MoveClient>   move_client_;
  std::unique_ptr<GraspClient>  grasp_client_;
  std::unique_ptr<StopClient>   stop_client_;

  bool robot_reached_target_;                                        
  GripperState gripper_state_;
  bool gripper_cmd_sent_;
  ros::ServiceServer release_srv_;
  bool released_;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
  // ros::Subscriber release_signal_sub_;
  bool release_requested_;
  // void releaseCallback(const std_msgs::Bool::ConstPtr& msg);
  bool releaseServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
};

}  // namespace franka_example_controllers
