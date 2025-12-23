// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id="panda";
  // if (!node_handle.getParam("arm_id", arm_id)) {
  //   ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
  //   return false;
  // }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));

    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  release_srv_ = node_handle.advertiseService("gripper_release", &JointVelocityExampleController::releaseServiceCallback, this);

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

  robot_reached_target_ = false;
  gripper_state_ = GripperState::OPEN;
  gripper_cmd_sent_ = false;
  release_requested_ = false;
  released_ = false;

  return true;
}

void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void JointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // const std::array<double, 7> q_target{{1.22020739, -0.86006264, -1.37826989, -2.07608879, -0.1665989, 3.38659885, 0.10734876}};
  const std::array<double, 7> q_target{{2.23832222, -0.95686087, -1.87890471, -0.8433237,  -0.48770463,  2.9228949, 0.36379326}};

  ros::Duration time_max(8.0);

  const auto& robot_state = state_handle_->getRobotState();

  double max_e = 0.0;
  double max_dq = 0.0;

  for (size_t i = 0; i < 7; ++i) {
    double q = robot_state.q[i];
    double dq = robot_state.dq[i];
    double e = q_target[i] - q;

    max_e = std::max(max_e, std::abs(e));
    max_dq = std::max(max_dq, std::abs(dq));

    double omega_cmd = omega_max * std::tanh(kp_ * e);
    velocity_joint_handles_[i].setCommand(omega_cmd);
  }
  // ROS_INFO("max_e = %.6f, max_dq = %.6f", max_e,max_dq);

  if (max_e < e_tol_ && max_dq < dq_tol_) {
    stable_time_ += period;
    if (stable_time_.toSec() > stable_duration_) {
      robot_reached_target_ = true;
    }
  } else {
    stable_time_ = ros::Duration(0.0);
  }

  if (robot_reached_target_)
  {
    switch (gripper_state_)
    {
      case GripperState::OPEN:
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
        if (release_requested_) 
        {
          ROS_INFO("Release Signal Received!");
          gripper_state_ = GripperState::RELEASE;
        }
        break;

      case GripperState::RELEASE:
        if (release_requested_ && !released_) {
          // stop_client_->sendGoal(franka_gripper::StopGoal());

          franka_gripper::MoveGoal goal;
          goal.width = 0.08;
          goal.speed = 0.3;
          move_client_->sendGoal(goal);
          released_ = true;
        }
        break;

      default:
        break;
    }
  }
}

bool JointVelocityExampleController::releaseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    release_requested_ = true;

    res.success = true;
    res.message = "Gripper release requested";

    ROS_INFO("gripper_release service called");
    return true;
}

void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
