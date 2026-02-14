#include <franka_gazebo/joint.h>
#include <urdf/model.h>
#include <gazebo/physics/Link.hh>

namespace franka_gazebo {

void Joint::update(const ros::Duration& dt) {
  if (not this->handle) {
    return;
  }

  // Apply 'setJointPosition' position request.
  if (this->setPositionRequested_) {
    std::lock_guard<std::mutex> lock(this->requestedPositionMutex_);
    this->position = this->requestedPosition_;
    this->desired_position = this->requestedPosition_;
    this->stop_position = this->requestedPosition_;
    this->setPositionRequested_ = false;
  }

  this->velocity = this->handle->GetVelocity(0);
#if GAZEBO_MAJOR_VERSION >= 8
  double position = this->handle->Position(0);
#else
  double position = this->handle->GetAngle(0).Radian();
#endif
  ignition::math::Vector3d f;
  switch (this->type) {
    case urdf::Joint::PRISMATIC:
      this->position = position;
#if GAZEBO_MAJOR_VERSION >= 8
      f = this->handle->GetForceTorque(0).body2Force;
#else
      f = this->handle->GetForceTorque(0).body2Force.Ign();
#endif
      break;
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
      this->position += angles::shortest_angular_distance(this->position, position);
#if GAZEBO_MAJOR_VERSION >= 8
      f = this->handle->GetForceTorque(0).body2Torque;
#else
      f = this->handle->GetForceTorque(0).body2Torque.Ign();
#endif
      break;
    default:
      throw std::logic_error("Unknown joint type: " + std::to_string(this->type));
  }
  this->effort = Eigen::Vector3d(f.X(), f.Y(), f.Z()).dot(this->axis);

  if (std::isnan(this->lastVelocity)) {
    this->lastVelocity = this->velocity;
  }
  this->acceleration = (this->velocity - this->lastVelocity) / dt.toSec();
  this->lastVelocity = this->velocity;

  if (std::isnan(this->lastAcceleration)) {
    this->lastAcceleration = this->acceleration;
  }
  this->jerk = (this->acceleration - this->lastAcceleration) / dt.toSec();
  this->lastAcceleration = this->acceleration;
}

double Joint::getDesiredPosition(const franka::RobotMode& mode) const {
  if (mode != franka::RobotMode::kMove) {
    return this->position;
  }
  if (this->control_method == ControlMethod::POSITION or
      this->control_method != ControlMethod::EFFORT) {
    return this->desired_position;
  }
  return this->position;
}

double Joint::getDesiredVelocity(const franka::RobotMode& mode) const {
  if (mode != franka::RobotMode::kMove) {
    return this->velocity;
  }
  if (this->control_method != ControlMethod::VELOCITY) {
    return this->velocity;
  }
  return this->desired_velocity;
}

double Joint::getDesiredAcceleration(const franka::RobotMode& mode) const {
  if (mode != franka::RobotMode::kMove) {
    return this->acceleration;
  }
  if (this->control_method == ControlMethod::EFFORT) {
    return 0;
  }
  return this->acceleration;
}

double Joint::getDesiredTorque(const franka::RobotMode& mode) const {
  if (mode != franka::RobotMode::kMove) {
    return 0;
  }
  if (this->control_method != ControlMethod::EFFORT) {
    return 0;
  }
  return command;
}

double Joint::getLinkMass() const {
  if (not this->handle) {
    return std::numeric_limits<double>::quiet_NaN();
  }
#if GAZEBO_MAJOR_VERSION >= 8
  return this->handle->GetChild()->GetInertial()->Mass();
#else
  return this->handle->GetChild()->GetInertial()->GetMass();
#endif
}

bool Joint::isInCollision() const {
  return std::abs(this->effort - this->command + this->gravity) > this->collision_threshold;
}

bool Joint::isInContact() const {
  return std::abs(this->effort - this->command + this->gravity) > this->contact_threshold;
}

void Joint::setJointPosition(const double joint_position) {
  // NOTE: Joint position is set in update() method to prevent racing conditions.
  std::lock_guard<std::mutex> lock(this->requestedPositionMutex_);
  this->requestedPosition_ = joint_position;
  this->setPositionRequested_ = true;
}

}  // namespace franka_gazebo
