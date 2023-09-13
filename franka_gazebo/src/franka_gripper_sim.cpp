#include <cmath>
#include <memory>

#include <franka_gazebo/franka_gripper_sim.h>
#include <pluginlib/class_list_macros.h>

using namespace boost::placeholders;

namespace franka_gazebo {

using actionlib::SimpleActionServer;
using control_msgs::GripperCommandAction;
using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;

bool FrankaGripperSim::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  std::string ns = nh.getNamespace();
  std::string arm_id;

  if (not nh.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM_NAMED("FrankaGripperSim",
                           "Could not find required parameter '" << ns << "/arm_id'");
    return false;
  }
  std::string finger1 = arm_id + "_finger_joint1";
  std::string finger2 = arm_id + "_finger_joint2";

  nh.param<double>("move/width_tolerance", this->tolerance_move_, kDefaultMoveWidthTolerance);
  nh.param<double>("gripper_action/width_tolerance", this->tolerance_gripper_action_,
                   kDefaultGripperActionWidthTolerance);
  nh.param<double>("gripper_action/speed", this->speed_default_, kDefaultGripperActionSpeed);
  nh.param<double>("grasp/resting_threshold", this->speed_threshold_, kGraspRestingThreshold);
  nh.param<int>("grasp/consecutive_samples", this->speed_samples_, kGraspConsecutiveSamples);

  try {
    this->finger1_ = hw->getHandle(finger1);
    this->finger2_ = hw->getHandle(finger2);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM_NAMED("FrankaGripperSim", "Could not get joint handle(s): " << ex.what());
    return false;
  }

  if (not this->pid1_.initParam(ns + "/finger1/gains")) {
    return false;
  }
  if (not this->pid2_.initParam(ns + "/finger2/gains")) {
    return false;
  }

  pub_.init(nh, "joint_states", 1);
  pub_.lock();
  pub_.msg_.name = {finger1, finger2};
  pub_.unlock();

  this->action_stop_ = std::make_unique<SimpleActionServer<StopAction>>(
      nh, "stop", boost::bind(&FrankaGripperSim::onStopGoal, this, _1), false);
  this->action_stop_->start();

  this->action_homing_ = std::make_unique<SimpleActionServer<HomingAction>>(
      nh, "homing", boost::bind(&FrankaGripperSim::onHomingGoal, this, _1), false);
  this->action_homing_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Homing Action cancelled");
    this->setState(State::IDLE);
  });
  this->action_homing_->start();

  this->action_move_ = std::make_unique<SimpleActionServer<MoveAction>>(
      nh, "move", boost::bind(&FrankaGripperSim::onMoveGoal, this, _1), false);
  this->action_move_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Moving Action cancelled");
    this->setState(State::IDLE);
  });
  this->action_move_->start();

  this->action_grasp_ = std::make_unique<SimpleActionServer<GraspAction>>(
      nh, "grasp", boost::bind(&FrankaGripperSim::onGraspGoal, this, _1), false);
  this->action_grasp_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Grasping Action cancelled");
    this->setState(State::IDLE);
  });
  this->action_grasp_->start();

  this->action_gc_ = std::make_unique<SimpleActionServer<GripperCommandAction>>(
      nh, "gripper_action", boost::bind(&FrankaGripperSim::onGripperActionGoal, this, _1), false);
  this->action_gc_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Gripper Command Action cancelled");
    this->setState(State::IDLE);
  });
  this->action_gc_->start();

  ROS_INFO_STREAM_NAMED("FrankaGripperSim",
                        "Successfully initialized Franka Gripper Controller for joints '"
                            << finger1 << "' and '" << finger2 << "'");
  return true;
}

void FrankaGripperSim::starting(const ros::Time& /*unused*/) {
  transition(State::IDLE, Config());
  this->pid1_.reset();
  this->pid2_.reset();
}

void FrankaGripperSim::update(const ros::Time& now, const ros::Duration& period) {
  if (rate_trigger_() and pub_.trylock()) {
    pub_.msg_.header.stamp = now;
    pub_.msg_.position = {this->finger1_.getPosition(), this->finger2_.getPosition()};
    pub_.msg_.velocity = {this->finger1_.getVelocity(), this->finger2_.getVelocity()};
    pub_.msg_.effort = {this->finger1_.getEffort(), this->finger2_.getEffort()};
    pub_.unlockAndPublish();
  }

  // Read state threadsafe
  double width = this->finger1_.getPosition() + this->finger2_.getPosition();
  this->mutex_.lock();
  State state = this->state_;
  auto tolerance = this->config_.tolerance;
  double w_d = this->config_.width_desired;
  double dw_d = this->config_.speed_desired * std::copysign(1.0, w_d - width);
  this->mutex_.unlock();
  if (state == State::IDLE) {
    // Track position of other finger to simulate mimicked joints + high damping
    control(this->finger1_, this->pid1_, this->finger2_.getPosition(), 0, 0, period);
    control(this->finger2_, this->pid2_, this->finger1_.getPosition(), 0, 0, period);
    return;
  }

  // Compute control signal and send to joints
  double w1_d = this->finger1_.getPosition() + 0.5 * dw_d * period.toSec();
  double w2_d = this->finger2_.getPosition() + 0.5 * dw_d * period.toSec();

  // Only in case when we hold we want to add the desired force, in any other state don't add
  // anything extra to the command
  double f_d = 0;

  if (state == State::HOLDING) {
    // When an object is grasped, next to the force to apply, also track the other finger
    // to not make both fingers drift away from middle simultaneously
    w1_d = this->finger2_.getPosition();
    w2_d = this->finger1_.getPosition();
    std::lock_guard<std::mutex> lock(this->mutex_);
    f_d = this->config_.force_desired / 2.0;
  }

  control(this->finger1_, this->pid1_, w1_d, 0.5 * dw_d, f_d, period);
  control(this->finger2_, this->pid2_, w2_d, 0.5 * dw_d, f_d, period);

  if (w_d - tolerance.inner < width and width < w_d + tolerance.outer) {
    // Goal reached, update statemachine
    if (state == State::MOVING) {
      // Done with move motion, switch to idle again
      transition(State::IDLE, Config{.width_desired = this->config_.width_desired,
                                     .speed_desired = 0,
                                     .force_desired = 0,
                                     .tolerance = this->config_.tolerance});
      return;
    }
  }

  if (state == State::GRASPING or state == State::MOVING) {
    // Since the velocity signal is noisy it can easily happen that one sample is below the
    // threshold To avoid abortion because of noise, we have to read at least N consecutive number
    // of samples before interpreting something was grasped (or not)
    static int speed_threshold_counter = 0;
    double speed = this->finger1_.getVelocity() + this->finger2_.getVelocity();
    if (std::abs(speed) <= this->speed_threshold_) {
      speed_threshold_counter++;
    } else {
      speed_threshold_counter = 0;
    }

    if (speed_threshold_counter >= this->speed_samples_) {
      if (state == State::GRASPING) {
        // Done with grasp motion, switch to holding, i.e. keep position & force
        transition(State::HOLDING, Config{.width_desired = width,
                                          .speed_desired = 0,
                                          .force_desired = this->config_.force_desired,
                                          .tolerance = this->config_.tolerance});
      } else {
        // Moving failed due to object between fingers. Switch to idle.
        transition(State::IDLE, Config{.width_desired = width,
                                       .speed_desired = 0,
                                       .force_desired = 0,
                                       .tolerance = this->config_.tolerance});
      }
      speed_threshold_counter = 0;
    }
  }
}

void FrankaGripperSim::control(hardware_interface::JointHandle& joint,
                               control_toolbox::Pid& pid,
                               double q_d,
                               double dq_d,
                               double f_d,
                               const ros::Duration& period) {
  double error = q_d - joint.getPosition();
  double derror = dq_d - joint.getVelocity();
  joint.setCommand(pid.computeCommand(error, derror, period) + f_d);
}

void FrankaGripperSim::setState(const State&& state) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->state_ = state;
}

void FrankaGripperSim::setConfig(const Config&& config) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->config_ = config;
}

void FrankaGripperSim::transition(const State&& state, const Config&& config) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->state_ = state;
  this->config_ = config;
}

void FrankaGripperSim::interrupt(const std::string& message, const State& except) {
  if (except != State::MOVING and this->action_move_ != nullptr and
      this->action_move_->isActive()) {
    franka_gripper::MoveResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = message;
    this->action_move_->setAborted(result, result.error);
  }
  if (except != State::GRASPING and this->action_grasp_ != nullptr and
      this->action_grasp_->isActive()) {
    franka_gripper::GraspResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = message;
    this->action_grasp_->setAborted(result, result.error);
  }
  if (except != State::MOVING and this->action_homing_ != nullptr and
      this->action_homing_->isActive()) {
    franka_gripper::HomingResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = message;
    this->action_homing_->setAborted(result, result.error);
  }
}

void FrankaGripperSim::waitUntilStateChange() {
  State original = this->state_;  // copy

  ros::Rate rate(30);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->state_ != original) {
        return;
      }
    }
    rate.sleep();
  }
}

void FrankaGripperSim::onStopGoal(const franka_gripper::StopGoalConstPtr& /*goal*/) {
  ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Stop Action goal received");

  interrupt("Command interrupted, because stop action was called", State::IDLE);

  transition(State::IDLE, Config{.width_desired = this->config_.width_desired,
                                 .speed_desired = 0,
                                 .force_desired = 0,
                                 .tolerance = this->config_.tolerance});

  franka_gripper::StopResult result;
  result.success = static_cast<decltype(result.success)>(true);
  action_stop_->setSucceeded(result);
}

void FrankaGripperSim::onHomingGoal(const franka_gripper::HomingGoalConstPtr& /*goal*/) {
  ROS_INFO_STREAM_NAMED("FrankaGripperSim", "New Homing Action goal received");

  if (this->state_ != State::IDLE) {
    this->interrupt("Command interrupted, because new homing action called", State::MOVING);
  }

  franka_gripper::GraspEpsilon eps;
  eps.inner = this->tolerance_move_;
  eps.outer = this->tolerance_move_;
  transition(
      State::MOVING,
      Config{.width_desired = 0, .speed_desired = 0.02, .force_desired = 0, .tolerance = eps});

  waitUntilStateChange();

  if (not this->action_homing_->isActive()) {
    // Homing Action was interrupted from another action goal callback and already preempted.
    // Don't try to resend result now
    return;
  }
  transition(State::MOVING, Config{.width_desired = kMaxFingerWidth,
                                   .speed_desired = 0.02,
                                   .force_desired = 0,
                                   .tolerance = eps});

  waitUntilStateChange();
  if (not this->action_homing_->isActive()) {
    // Homing Action was interrupted from another action goal callback and already preempted.
    // Don't try to resend result now
    return;
  }

  franka_gripper::HomingResult result;
  if (this->state_ != State::IDLE) {
    result.success = static_cast<decltype(result.success)>(false);
    result.error = "Unexpected state transistion: The gripper not in IDLE as expected";
    action_homing_->setAborted(result, result.error);
    return;
  }

  result.success = static_cast<decltype(result.success)>(true);
  action_homing_->setSucceeded(result);
}

void FrankaGripperSim::onMoveGoal(const franka_gripper::MoveGoalConstPtr& goal) {
  ROS_INFO_STREAM_NAMED("FrankaGripperSim",
                        "New Move Action Goal received: " << goal->width << " m");
  if (goal->speed < 0) {
    franka_gripper::MoveResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = "Only positive speeds allowed";
    action_move_->setAborted(result, result.error);
    return;
  }

  if (goal->width < 0 or goal->width > kMaxFingerWidth or not std::isfinite(goal->width)) {
    franka_gripper::MoveResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = "Target width has to lie between 0 .. " + std::to_string(kMaxFingerWidth);
    action_move_->setAborted(result, result.error);
    return;
  }

  if (this->state_ != State::IDLE) {
    interrupt("Command interrupted, because new move action called", State::MOVING);
  }

  bool move_succeeded = move(goal->width, goal->speed);

  if (not this->action_move_->isActive()) {
    // Move Action was interrupted from another action goal callback and already preempted.
    // Don't try to resend result now
    return;
  }

  franka_gripper::MoveResult result;
  if (not move_succeeded) {
    result.success = static_cast<decltype(result.success)>(false);
    result.error = "Unexpected state transistion: The gripper not in IDLE as expected";
    action_move_->setAborted(result, result.error);
    return;
  }
  franka_gripper::GraspEpsilon eps;
  eps.inner = this->tolerance_move_;
  eps.outer = this->tolerance_move_;

  double width = this->finger1_.getPosition() + this->finger2_.getPosition();  // recalculate
  bool ok = goal->width - eps.inner < width and width < goal->width + eps.outer;
  result.success = static_cast<decltype(result.success)>(ok);
  action_move_->setSucceeded(result);
}

void FrankaGripperSim::onGraspGoal(const franka_gripper::GraspGoalConstPtr& goal) {
  ROS_INFO_STREAM_NAMED("FrankaGripperSim",
                        "New Grasp Action Goal received: " << goal->force << "N");

  if (goal->width >= kMaxFingerWidth or goal->width < 0) {
    franka_gripper::GraspResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error =
        "Can only grasp inside finger width from [0 .. " + std::to_string(kMaxFingerWidth) + "[";
    action_grasp_->setAborted(result, result.error);
    return;
  }
  if (goal->speed < 0) {
    franka_gripper::GraspResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = "Only positive speeds allowed";
    action_grasp_->setAborted(result, result.error);
    return;
  }

  if (this->state_ != State::IDLE) {
    interrupt("Command interrupted, because new grasp action called", State::GRASPING);
  }

  bool grasp_succeeded = grasp(goal->width, goal->speed, goal->force, goal->epsilon);

  if (not this->action_grasp_->isActive()) {
    // Grasping Action was interrupted from another action goal callback and already preempted.
    // Don't try to resend result now
    return;
  }

  franka_gripper::GraspResult result;
  if (this->state_ != State::HOLDING) {
    result.success = static_cast<decltype(result.success)>(false);
    result.error = "Unexpected state transistion: The gripper not in HOLDING as expected";
    action_grasp_->setAborted(result, result.error);
    return;
  }

  result.success = static_cast<decltype(result.success)>(grasp_succeeded);
  if (not grasp_succeeded) {
    double current_width = this->finger1_.getPosition() + this->finger2_.getPosition();
    result.error =
        "When the gripper stopped (below speed of " + std::to_string(this->speed_threshold_) +
        " m/s the width between the fingers was not at " + std::to_string(goal->width) + "m (-" +
        std::to_string(goal->epsilon.inner) + "m/+" + std::to_string(goal->epsilon.outer) +
        "m) but at " + std::to_string(current_width) + "m";
    setState(State::IDLE);
  }

  action_grasp_->setSucceeded(result);
}

void FrankaGripperSim::onGripperActionGoal(const control_msgs::GripperCommandGoalConstPtr& goal) {
  control_msgs::GripperCommandResult result;
  // HACK: As one gripper finger is <mimic>, MoveIt!'s trajectory execution manager
  // only sends us the width of one finger. Multiply by 2 to get the intended width.
  double width_d = goal->command.position * 2.0;

  ROS_INFO_STREAM_NAMED("FrankaGripperSim", "New Gripper Command Action Goal received: "
                                                << goal->command.position << "m, "
                                                << goal->command.max_effort << "N");

  if (width_d > kMaxFingerWidth || width_d < 0.0) {
    std::string error =
        "Commanding out of range position! max_position = " + std::to_string(kMaxFingerWidth / 2) +
        ", commanded position = " + std::to_string(goal->command.position) +
        ". Be aware that you command the position of"
        " each finger which is half of the total opening width!";
    ROS_ERROR_STREAM_NAMED("FrankaGripperSim", error);
    result.reached_goal = static_cast<decltype(result.reached_goal)>(false);
    action_gc_->setAborted(result, error);
    return;
  }

  franka_gripper::GraspEpsilon eps;
  eps.inner = this->tolerance_gripper_action_;
  eps.outer = this->tolerance_gripper_action_;

  double current_width = this->finger1_.getPosition() + this->finger2_.getPosition();
  constexpr double kMinimumGraspForce = 1e-4;
  bool succeeded = false;

  if (std::abs(goal->command.max_effort) < kMinimumGraspForce or width_d > current_width) {
    succeeded = move(width_d, this->speed_default_);
    if (not this->action_gc_->isActive()) {
      // Gripper Action was interrupted from another action goal callback and already preempted.
      // Don't try to resend result now
      return;
    }
  } else {
    succeeded = grasp(width_d, this->speed_default_, goal->command.max_effort, eps);
    if (not this->action_gc_->isActive()) {
      // Gripper Action was interrupted from another action goal callback and already preempted.
      // Don't try to resend result now
      return;
    }
    if (this->state_ != State::HOLDING) {
      result.reached_goal = static_cast<decltype(result.reached_goal)>(false);
      std::string error = "Unexpected state transition: The gripper not in HOLDING as expected";
      action_gc_->setAborted(result, error);
      return;
    }
  }

  result.position = this->finger1_.getPosition() + this->finger2_.getPosition();
  result.effort = 0;
  result.stalled = static_cast<decltype(result.stalled)>(false);
  result.reached_goal = static_cast<decltype(result.reached_goal)>(succeeded);
  if (not succeeded) {
    setState(State::IDLE);
  }
  action_gc_->setSucceeded(result);
}

bool FrankaGripperSim::move(double width, double speed) {
  franka_gripper::GraspEpsilon eps;
  eps.inner = this->tolerance_move_;
  eps.outer = this->tolerance_move_;
  transition(
      State::MOVING,
      Config{.width_desired = width, .speed_desired = speed, .force_desired = 0, .tolerance = eps});

  waitUntilStateChange();
  return this->state_ == State::IDLE;
}

bool FrankaGripperSim::grasp(double width,
                             double speed,
                             double force,
                             const franka_gripper::GraspEpsilon& epsilon) {
  double current_width = this->finger1_.getPosition() + this->finger2_.getPosition();
  double direction = std::copysign(1.0, width - current_width);
  transition(State::GRASPING, Config{.width_desired = width < current_width ? 0 : kMaxFingerWidth,
                                     .speed_desired = speed,
                                     .force_desired = direction * force,
                                     .tolerance = epsilon});

  waitUntilStateChange();
  current_width = this->finger1_.getPosition() + this->finger2_.getPosition();  // recalculate
  return width - epsilon.inner < current_width and current_width < width + epsilon.outer;
}

}  // namespace franka_gazebo

PLUGINLIB_EXPORT_CLASS(franka_gazebo::FrankaGripperSim, controller_interface::ControllerBase);
