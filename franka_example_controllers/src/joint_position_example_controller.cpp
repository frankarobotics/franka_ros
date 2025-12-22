// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// ===== Action client typedefs =====
using HomingClient = actionlib::SimpleActionClient<franka_gripper::HomingAction>;
using MoveClient   = actionlib::SimpleActionClient<franka_gripper::MoveAction>;
using GraspClient  = actionlib::SimpleActionClient<franka_gripper::GraspAction>;
using StopClient   = actionlib::SimpleActionClient<franka_gripper::StopAction>;

namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  release_signal_sub_ = node_handle.subscribe("gripper_release", 1, &JointPositionExampleController::releaseCallback, this);

  robot_reached_target_ = false;

  homing_client_ = std::make_unique<HomingClient>("/franka_gripper/homing", true);
  move_client_   = std::make_unique<MoveClient>  ("/franka_gripper/move",   true);
  grasp_client_  = std::make_unique<GraspClient> ("/franka_gripper/grasp",  true);
  stop_client_   = std::make_unique<StopClient>  ("/franka_gripper/stop",   true);

  homing_client_->waitForServer();
  move_client_->waitForServer();
  grasp_client_->waitForServer();
  stop_client_->waitForServer();

  franka_gripper::HomingGoal goal;
  homing_client_->sendGoal(goal);
  homing_client_->waitForResult();

  gripper_state_ = GripperState::OPEN;
  gripper_cmd_sent_ = false;
  this->release_requested_ = false;
  released_ = false;

  return true;
}

void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  const std::array<double, 7> q_target{{1.22020739, -0.86006264, -1.37826989, -2.07608879, -0.1665989, 3.38659885, 0.10734876}};
  const double motion_time = 8.0;

  double s = elapsed_time_.toSec() / motion_time;
  if (s > 1.0) 
  {
    s = 1.0;
    robot_reached_target_ = true;
  }
  for (size_t i = 0; i < 7; ++i) {
    double q_cmd =
        initial_pose_[i] + s * (q_target[i] - initial_pose_[i]);
    position_joint_handles_[i].setCommand(q_cmd);
  }
  if (robot_reached_target_)
  {
    switch (gripper_state_)
    {
      case GripperState::OPEN:
        // ROS_INFO("OPEN");
        if (!gripper_cmd_sent_) 
        {
          franka_gripper::GraspGoal goal;
          goal.width = 0.01;
          goal.speed = 0.01;
          goal.force = 8.0;
          grasp_client_->sendGoal(goal);

          gripper_cmd_sent_ = true;
          gripper_state_ = GripperState::GRASP;
        }
        break;
      
      case GripperState::GRASP:
        // ROS_INFO("GRASP");
        if (this->release_requested_) 
        {
          ROS_INFO("Release Signal Received!");
          gripper_state_ = GripperState::RELEASE;
        }
        break;

      case GripperState::RELEASE:
        // ROS_INFO("RELEASE");
        if (this->release_requested_ && !released_) {
          // stop_client_->sendGoal(franka_gripper::StopGoal());

          franka_gripper::MoveGoal goal;
          goal.width = 0.08;
          goal.speed = 0.15;
          move_client_->sendGoal(goal);
          released_ = true;
        }
        break;
      
      default:
        break;
    }
  }
}

void JointPositionExampleController::releaseCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ROS_INFO("topic received!");
        this->release_requested_ = true;
    }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
