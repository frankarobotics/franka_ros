// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_wall.h>
#include <franka_example_controllers/teleop_joint_pd_example_controller.h>

#include <hardware_interface/hardware_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

using Vector7d = Eigen::Matrix<double, 7, 1>;
using boost::placeholders::_1;
using boost::placeholders::_2;

const std::string kControllerName = "TeleopJointPDExampleController";

namespace franka_example_controllers {

bool TeleopJointPDExampleController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  std::string leader_arm_id;
  std::string follower_arm_id;

  std::vector<std::string> leader_joint_names;
  std::vector<std::string> follower_joint_names;

  try {
    k_d_leader_lower_ = get7dParam("leader/d_gains_lower", node_handle);
    k_d_leader_upper_ = get7dParam("leader/d_gains_upper", node_handle);
    dq_max_leader_lower_ = get7dParam("leader/dq_max_lower", node_handle);
    dq_max_leader_upper_ = get7dParam("leader/dq_max_upper", node_handle);

    k_p_follower_ = get7dParam("follower/p_gains", node_handle);
    k_d_follower_ = get7dParam("follower/d_gains", node_handle);
    k_dq_ = get7dParam("follower/drift_comp_gains", node_handle);
    dq_max_lower_ = get7dParam("follower/dq_max_lower", node_handle);
    dq_max_upper_ = get7dParam("follower/dq_max_upper", node_handle);
    ddq_max_lower_ = get7dParam("follower/ddq_max_lower", node_handle);
    ddq_max_upper_ = get7dParam("follower/ddq_max_upper", node_handle);

    leader_data_.contact_force_threshold =
        get1dParam<double>("leader/contact_force_threshold", node_handle);
    follower_data_.contact_force_threshold =
        get1dParam<double>("follower/contact_force_threshold", node_handle);

    leader_arm_id = get1dParam<std::string>("leader/arm_id", node_handle);
    follower_arm_id = get1dParam<std::string>("follower/arm_id", node_handle);

    leader_joint_names = getJointParams<std::string>("leader/joint_names", node_handle);
    follower_joint_names = getJointParams<std::string>("follower/joint_names", node_handle);

    if (!node_handle.getParam("debug", debug_)) {
      ROS_INFO_STREAM_NAMED(kControllerName, "Could not find parameter debug. Defaulting to "
                                                 << std::boolalpha << debug_);
    }

    // Init for each arm
    initArm(robot_hw, node_handle, leader_data_, leader_arm_id, leader_joint_names);
    initArm(robot_hw, node_handle, follower_data_, follower_arm_id, follower_joint_names);
  } catch (const std::invalid_argument& ex) {
    ROS_ERROR_NAMED(kControllerName, "%s", ex.what());
    return false;
  }

  if (debug_) {
    // Init for dynamic reconfigure
    dynamic_reconfigure_teleop_param_node_ = ros::NodeHandle("dyn_reconf_teleop_param_node");
    dynamic_server_teleop_param_ = std::make_unique<
        dynamic_reconfigure::Server<franka_example_controllers::teleop_paramConfig>>(
        dynamic_reconfigure_teleop_param_node_);
    dynamic_server_teleop_param_->setCallback(
        boost::bind(&TeleopJointPDExampleController::teleopParamCallback, this, _1, _2));

    // Init for publishers
    auto init_publisher = [&node_handle](auto& publisher, const auto& topic) {
      publisher.init(node_handle, topic, 1);
      publisher.lock();
      publisher.msg_.name.resize(7);
      publisher.msg_.position.resize(7);
      publisher.msg_.velocity.resize(7);
      publisher.msg_.effort.resize(7);
      publisher.unlock();
    };

    init_publisher(leader_target_pub_, "leader_target");
    init_publisher(follower_target_pub_, "follower_target");
    leader_contact_pub_.init(node_handle, "leader_contact", 1);
    follower_contact_pub_.init(node_handle, "follower_contact", 1);
    marker_pub_.init(node_handle, "marker_labels", 1, true);

    auto get_marker = [](const std::string& arm_id, int32_t id, const std::string& text) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = arm_id + "_link0";
      marker.header.stamp = ros::Time::now();
      marker.ns = "basic_shapes";
      marker.id = id;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 1.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.text = text;

      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.1;

      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      return marker;
    };

    {
      std::lock_guard<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>> lock(
          marker_pub_);
      marker_pub_.msg_.markers.push_back(get_marker(leader_arm_id, 1, "leader"));
      marker_pub_.msg_.markers.push_back(get_marker(follower_arm_id, 2, "follower"));
    }
    publishMarkers();
  }

  return true;
}

void TeleopJointPDExampleController::initArm(hardware_interface::RobotHW* robot_hw,
                                             ros::NodeHandle& node_handle,
                                             FrankaDataContainer& arm_data,
                                             const std::string& arm_id,
                                             const std::vector<std::string>& joint_names) {
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (not effort_joint_interface) {
    throw std::invalid_argument(kControllerName +
                                ": Error getting effort joint interface from hardware of " +
                                arm_id + ".");
  }

  arm_data.joint_handles.clear();
  for (const auto& name : joint_names) {
    try {
      arm_data.joint_handles.push_back(effort_joint_interface->getHandle(name));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      throw std::invalid_argument(kControllerName +
                                  ": Exception getting joint handle: " + std::string(e.what()));
    }
  }

  // Get state interface.
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (not state_interface) {
    throw std::invalid_argument(kControllerName + ": Error getting state interface from hardware.");
  }

  try {
    arm_data.state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    throw std::invalid_argument(
        kControllerName +
        ": Exception getting state handle from interface: " + std::string(ex.what()));
  }

  // Setup joint walls
  // Virtual joint position wall parameters
  const std::array<double, 7> kPDZoneWidth = {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
  const std::array<double, 7> kDZoneWidth = {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
  const std::array<double, 7> kPDZoneStiffness = {
      {2000.0, 2000.0, 1000.0, 1000.0, 500.0, 200.0, 200.0}};
  const std::array<double, 7> kPDZoneDamping = {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};
  const std::array<double, 7> kDZoneDamping = {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};

  std::array<double, 7> upper_joint_soft_limit;
  std::array<double, 7> lower_joint_soft_limit;
  getJointLimits(node_handle, joint_names, upper_joint_soft_limit, lower_joint_soft_limit);

  arm_data.virtual_joint_wall = std::make_unique<JointWallContainer<7>>(
      upper_joint_soft_limit, lower_joint_soft_limit, kPDZoneWidth, kDZoneWidth, kPDZoneStiffness,
      kPDZoneDamping, kDZoneDamping);
}

void TeleopJointPDExampleController::starting(const ros::Time& /*time*/) {
  // Reset joint walls to start from the current q, dq
  leader_data_.virtual_joint_wall->reset();
  follower_data_.virtual_joint_wall->reset();

  // Reset stored states to the current states.
  franka::RobotState follower_robot_state = follower_data_.state_handle->getRobotState();
  q_target_last_ = Eigen::Map<Vector7d>(follower_robot_state.q.data());
  dq_target_last_.setZero();
  leader_data_.tau_target_last.setZero();
  follower_data_.tau_target_last.setZero();

  // Store alignment position from leader
  franka::RobotState leader_robot_state = leader_data_.state_handle->getRobotState();
  init_leader_q_ = Eigen::Map<Vector7d>(leader_robot_state.q.data());
  current_state_ = TeleopStateMachine::ALIGN;
  if (debug_) {
    publishMarkers();
  }
}

void TeleopJointPDExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  franka::RobotState leader_robot_state = leader_data_.state_handle->getRobotState();
  franka::RobotState follower_robot_state = follower_data_.state_handle->getRobotState();
  leader_data_.q = Eigen::Map<Vector7d>(leader_robot_state.q.data());
  leader_data_.dq = Eigen::Map<Vector7d>(leader_robot_state.dq.data());
  follower_data_.q = Eigen::Map<Vector7d>(follower_robot_state.q.data());
  follower_data_.dq = Eigen::Map<Vector7d>(follower_robot_state.dq.data());

  if (current_state_ == TeleopStateMachine::ALIGN) {
    // Check coefficient-wise if the two robots are aligned
    const auto kNorm = (leader_data_.q - follower_data_.q).cwiseAbs().array();
    if ((kNorm < kAlignmentTolerance_).all()) {
      current_state_ = TeleopStateMachine::TRACK;
      ROS_INFO_STREAM_NAMED(kControllerName, "Leader and follower are aligned");
    }
  }

  // Determine contact scaling factor depending on the external cartesian forces applied on the
  // endeffector.
  Vector6d leader_f_ext_hat = Eigen::Map<Vector6d>(leader_robot_state.K_F_ext_hat_K.data());
  leader_data_.f_ext_norm = leader_f_ext_hat.head(3).norm();
  leader_data_.contact =
      rampParameter(leader_data_.f_ext_norm, 1.0, 0.0, leader_data_.contact_force_threshold,
                    leader_data_.contact_ramp_increase);

  Vector6d follower_f_ext_hat = Eigen::Map<Vector6d>(follower_robot_state.K_F_ext_hat_K.data());
  follower_data_.f_ext_norm = follower_f_ext_hat.head(3).norm();
  follower_data_.contact =
      rampParameter(follower_data_.f_ext_norm, 1.0, 0.0, follower_data_.contact_force_threshold,
                    follower_data_.contact_ramp_increase);

  // Determine max velocities and accelerations of follower arm depending on tracking errors to
  // avoid jumps and high velocities when starting example.
  Vector7d q_deviation = (q_target_last_ - leader_data_.q).cwiseAbs();
  Vector7d dq_max;
  Vector7d ddq_max;

  if (current_state_ == TeleopStateMachine::ALIGN) {
    dq_max = dq_max_align_;
    ddq_max = ddq_max_align_;
    prev_alignment_error_ = alignment_error_;
    alignment_error_ = (init_leader_q_ - follower_data_.q);
    Vector7d dalignment_error = (alignment_error_ - prev_alignment_error_) / period.toSec();

    dq_unsaturated_ = k_p_follower_align_.asDiagonal() * alignment_error_ +
                      k_d_follower_align_.asDiagonal() * dalignment_error;
  } else {
    for (size_t i = 0; i < 7; ++i) {
      dq_max[i] = rampParameter(q_deviation[i], dq_max_lower_[i], dq_max_upper_[i],
                                velocity_ramp_shift_, velocity_ramp_increase_);
      ddq_max[i] = rampParameter(q_deviation[i], ddq_max_lower_[i], ddq_max_upper_[i],
                                 velocity_ramp_shift_, velocity_ramp_increase_);
    }
    dq_unsaturated_ = k_dq_.asDiagonal() * (leader_data_.q - q_target_last_) + leader_data_.dq;
  }

  // Calculate target postions and velocities for follower arm
  dq_target_ = saturateAndLimit(dq_unsaturated_, dq_target_last_, dq_max, ddq_max, period.toSec());
  dq_target_last_ = dq_target_;
  q_target_ = q_target_last_ + (dq_target_ * period.toSec());
  q_target_last_ = q_target_;

  if (!leader_robot_state.current_errors && !follower_robot_state.current_errors) {
    if (current_state_ == TeleopStateMachine::ALIGN) {
      // Compute P control for the leader arm to stay in position.
      leader_data_.tau_target = k_p_follower_.asDiagonal() * (init_leader_q_ - leader_data_.q);
    } else {
      // Compute force-feedback for the leader arm to render the haptic interaction of the follower
      // robot. Add a slight damping to reduce vibrations.
      // The force feedback is applied when the external forces on the follower arm exceed a
      // threshold. While the leader arm is unguided (not in contact), the force-feedback is
      // reduced. When the leader robot exceeds the soft limit velocities dq_max_leader_lower
      // damping is increased gradually until it saturates when reaching dq_max_leader_upper to
      // maximum damping.

      Vector7d follower_tau_ext_hat =
          Eigen::Map<Vector7d>(follower_robot_state.tau_ext_hat_filtered.data());
      Vector7d leader_damping_torque =
          leader_damping_scaling_ * leaderDamping(leader_data_.dq).asDiagonal() * leader_data_.dq;
      Vector7d leader_force_feedback =
          follower_data_.contact *
          (force_feedback_idle_ +
           leader_data_.contact * (force_feedback_guiding_ - force_feedback_idle_)) *
          (-follower_tau_ext_hat);

      leader_data_.tau_target = leader_force_feedback - leader_damping_torque;
    }

    // Compute PD control for the follower arm to track the leader's motions.
    follower_data_.tau_target =
        follower_stiffness_scaling_ * k_p_follower_.asDiagonal() * (q_target_ - follower_data_.q) +
        sqrt(follower_stiffness_scaling_) * k_d_follower_.asDiagonal() *
            (dq_target_ - follower_data_.dq);
  } else {
    // Control target torques to zero if any arm is in error state.
    leader_data_.tau_target = decrease_factor_ * leader_data_.tau_target_last;
    follower_data_.tau_target = decrease_factor_ * follower_data_.tau_target_last;
  }

  // Add torques from joint walls to the torque commands.
  auto to_eigen = [](const std::array<double, 7>& data) { return Vector7d(data.data()); };
  auto from_eigen = [](const Vector7d& data) {
    return std::array<double, 7>{data(0), data(1), data(2), data(3), data(4), data(5), data(6)};
  };
  std::array<double, 7> virtual_wall_tau_leader = leader_data_.virtual_joint_wall->computeTorque(
      from_eigen(leader_data_.q), from_eigen(leader_data_.dq));

  std::array<double, 7> virtual_wall_tau_follower =
      follower_data_.virtual_joint_wall->computeTorque(from_eigen(follower_data_.q),
                                                       from_eigen(follower_data_.dq));

  leader_data_.tau_target += to_eigen(virtual_wall_tau_leader);
  follower_data_.tau_target += to_eigen(virtual_wall_tau_follower);

  // Store torques for next time step
  leader_data_.tau_target_last = leader_data_.tau_target;
  follower_data_.tau_target_last = follower_data_.tau_target;

  updateArm(leader_data_);
  updateArm(follower_data_);

  if (debug_ && publish_rate_()) {
    publishLeaderTarget();
    publishFollowerTarget();
    publishLeaderContact();
    publishFollowerContact();
  }
}

void TeleopJointPDExampleController::updateArm(FrankaDataContainer& arm_data) {
  for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles[i].setCommand(arm_data.tau_target[i]);
  }
}

Eigen::Matrix<double, 7, 1> TeleopJointPDExampleController::saturateAndLimit(const Vector7d& x_calc,
                                                                             const Vector7d& x_last,
                                                                             const Vector7d& x_max,
                                                                             const Vector7d& dx_max,
                                                                             const double delta_t) {
  Vector7d x_limited;
  for (size_t i = 0; i < 7; i++) {
    double delta_x_max = dx_max[i] * delta_t;
    double diff = x_calc[i] - x_last[i];
    double x_saturated = x_last[i] + std::max(std::min(diff, delta_x_max), -delta_x_max);
    x_limited[i] = std::max(std::min(x_saturated, x_max[i]), -x_max[i]);
  }
  return x_limited;
}

double TeleopJointPDExampleController::rampParameter(const double x,
                                                     const double neg_x_asymptote,
                                                     const double pos_x_asymptote,
                                                     const double shift_along_x,
                                                     const double increase_factor) {
  double ramp =
      0.5 * (pos_x_asymptote + neg_x_asymptote -
             (pos_x_asymptote - neg_x_asymptote) * tanh(increase_factor * (x - shift_along_x)));
  return ramp;
}

void TeleopJointPDExampleController::teleopParamCallback(
    franka_example_controllers::teleop_paramConfig& config,
    uint32_t /*level*/) {
  if (dynamic_reconfigure_mutex_.try_lock()) {
    leader_damping_scaling_ = config.leader_damping_scaling;
    follower_stiffness_scaling_ = config.follower_stiffness_scaling;
    force_feedback_guiding_ = config.force_feedback_guiding;
    force_feedback_idle_ = config.force_feedback_idle;
    follower_data_.contact_force_threshold = config.follower_contact_force_threshold;
    leader_data_.contact_force_threshold = config.leader_contact_force_threshold;

    dq_max_lower_[0] = config.dq_l_1;
    dq_max_lower_[1] = config.dq_l_2;
    dq_max_lower_[2] = config.dq_l_3;
    dq_max_lower_[3] = config.dq_l_4;
    dq_max_lower_[4] = config.dq_l_5;
    dq_max_lower_[5] = config.dq_l_6;
    dq_max_lower_[6] = config.dq_l_7;

    dq_max_upper_[0] = config.dq_u_1;
    dq_max_upper_[1] = config.dq_u_2;
    dq_max_upper_[2] = config.dq_u_3;
    dq_max_upper_[3] = config.dq_u_4;
    dq_max_upper_[4] = config.dq_u_5;
    dq_max_upper_[5] = config.dq_u_6;
    dq_max_upper_[6] = config.dq_u_7;

    ddq_max_lower_[0] = config.ddq_l_1;
    ddq_max_lower_[1] = config.ddq_l_2;
    ddq_max_lower_[2] = config.ddq_l_3;
    ddq_max_lower_[3] = config.ddq_l_4;
    ddq_max_lower_[4] = config.ddq_l_5;
    ddq_max_lower_[5] = config.ddq_l_6;
    ddq_max_lower_[6] = config.ddq_l_7;

    ddq_max_upper_[0] = config.ddq_u_1;
    ddq_max_upper_[1] = config.ddq_u_2;
    ddq_max_upper_[2] = config.ddq_u_3;
    ddq_max_upper_[3] = config.ddq_u_4;
    ddq_max_upper_[4] = config.ddq_u_5;
    ddq_max_upper_[5] = config.ddq_u_6;
    ddq_max_upper_[6] = config.ddq_u_7;

    ROS_INFO_NAMED(kControllerName, "Dynamic reconfigure: Controller params set.");
  }
  dynamic_reconfigure_mutex_.unlock();
}

void TeleopJointPDExampleController::getJointLimits(ros::NodeHandle& nh,
                                                    const std::vector<std::string>& joint_names,
                                                    std::array<double, 7>& upper_joint_soft_limit,
                                                    std::array<double, 7>& lower_joint_soft_limit) {
  const std::string& node_namespace = nh.getNamespace();
  std::size_t found = node_namespace.find_last_of('/');
  std::string parent_namespace = node_namespace.substr(0, found);

  if (!nh.hasParam(parent_namespace + "/robot_description")) {
    throw std::invalid_argument(kControllerName + ": No parameter robot_description (namespace: " +
                                parent_namespace + ")found to set joint limits!");
  }

  urdf::Model urdf_model;
  if (!urdf_model.initParamWithNodeHandle(parent_namespace + "/robot_description", nh)) {
    throw std::invalid_argument(kControllerName +
                                ": Could not initialize urdf model from robot_description "
                                "(namespace: " +
                                parent_namespace + ").");
  }
  joint_limits_interface::SoftJointLimits soft_limits;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string& joint_name = joint_names.at(i);
    auto urdf_joint = urdf_model.getJoint(joint_name);
    if (!urdf_joint) {
      ROS_ERROR_STREAM_NAMED(kControllerName,
                             ": Could not get joint " << joint_name << " from urdf");
    }
    if (!urdf_joint->safety) {
      ROS_ERROR_STREAM_NAMED(kControllerName, ": Joint " << joint_name << " has no limits");
    }
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
      upper_joint_soft_limit[i] = soft_limits.max_position;
      lower_joint_soft_limit[i] = soft_limits.min_position;
    } else {
      ROS_ERROR_STREAM_NAMED(kControllerName, ": Could not parse joint limit for joint "
                                                  << joint_name << " for joint limit interfaces");
    }
  }
}

void TeleopJointPDExampleController::publishLeaderTarget() {
  if (leader_target_pub_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      leader_target_pub_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      leader_target_pub_.msg_.position[i] = 0.0;
      leader_target_pub_.msg_.velocity[i] = 0.0;
      leader_target_pub_.msg_.effort[i] = leader_data_.tau_target[i];
    }
    leader_target_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishFollowerTarget() {
  if (follower_target_pub_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      follower_target_pub_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      follower_target_pub_.msg_.position[i] = q_target_[i];
      follower_target_pub_.msg_.velocity[i] = dq_target_[i];
      follower_target_pub_.msg_.effort[i] = follower_data_.tau_target[i];
    }
    follower_target_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishLeaderContact() {
  if (leader_contact_pub_.trylock()) {
    leader_contact_pub_.msg_.data = leader_data_.contact;
    leader_contact_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishFollowerContact() {
  if (follower_contact_pub_.trylock()) {
    follower_contact_pub_.msg_.data = follower_data_.contact;
    follower_contact_pub_.unlockAndPublish();
  }
}

Vector7d TeleopJointPDExampleController::get7dParam(const std::string& param_name,
                                                    ros::NodeHandle& nh) {
  auto buffer = getJointParams<double>(param_name, nh);
  return Vector7d(Eigen::Map<Vector7d>(buffer.data()));
}

Vector7d TeleopJointPDExampleController::leaderDamping(const Vector7d& dq) {
  auto simple_ramp = [](const double min, const double max, const double value) -> double {
    if (value >= max) {
      return 1.0;
    }
    if (value <= min) {
      return 0.0;
    }
    return (value - min) / (max - min);
  };
  Vector7d damping;
  for (size_t i = 0; i < 7; ++i) {
    damping(i) = k_d_leader_lower_(i) +
                 simple_ramp(dq_max_leader_lower_(i), dq_max_leader_upper_(i), std::abs(dq(i))) *
                     (k_d_leader_upper_(i) - k_d_leader_lower_(i));
  }
  return damping;
}

void TeleopJointPDExampleController::publishMarkers() {
  marker_pub_.lock();
  marker_pub_.unlockAndPublish();
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::TeleopJointPDExampleController,
                       controller_interface::ControllerBase)
