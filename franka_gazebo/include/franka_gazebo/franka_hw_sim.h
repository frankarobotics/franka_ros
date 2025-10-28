#pragma once

#include <actionlib/server/simple_action_server.h>
#include <franka/robot_state.h>
#include <franka_gazebo/controller_verifier.h>
#include <franka_gazebo/joint.h>
#include <franka_gazebo/statemachine.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/model_base.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <array>
#include <boost/optional.hpp>
#include <boost_sml/sml.hpp>
#include <cmath>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace franka_gazebo {

const double kDefaultTauExtLowpassFilter = 1.0;  // no filtering per default of tau_ext_hat_filtered
const std::array<std::string, 9> kRobotJointSuffixes = {
    "_joint1", "_joint2", "_joint3",        "_joint4",       "_joint5",
    "_joint6", "_joint7", "_finger_joint1", "_finger_joint2"};

/**
 * A custom implementation of a [gazebo_ros_control](http://wiki.ros.org/gazebo_ros_control) plugin,
 * which is able to simulate franka interfaces in Gazebo.
 *
 * Specifically it supports the following hardware transmission types
 *
 * ### transmission_interface/SimpleTransmission
 * - hardware_interface/JointStateInterface
 * - hardware_interface/EffortJointInterface
 * - hardware_interface/PositionJointInterface
 * - hardware_interface/VelocityJointInterface
 *
 * ### franka_hw/FrankaStateInterface
 * ### franka_hw/FrankaModelInterface
 *
 * @see
 * http://gazebosim.org/tutorials/?tut=ros_control#Advanced:customgazebo_ros_controlSimulationPlugins
 */
class FrankaHWSim : public gazebo_ros_control::RobotHWSim {
 public:
  /**
   * Create a new FrankaHWSim instance
   */
  FrankaHWSim();

  /**
   * Initialize the simulated robot hardware and parse all supported transmissions.
   *
   * @param[in] robot_namespace the name of the robot passed inside the `<robotNamespace>` tag from
   * the URDF
   * @param[in] model_nh root node handle of the node into which this plugin is loaded (usually
   * Gazebo)
   * @param[in] parent the underlying gazebo model type of the robot which was added
   * @param[in] urdf the parsed URDF which should be added
   * @param[in] transmissions a list of transmissions of the model which should be simulated
   * @return `true` if initialization succeeds, `false` otherwise
   */
  bool initSim(const std::string& robot_namespace,
               ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent,
               const urdf::Model* const urdf,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  /**
   * Fetch data from the Gazebo simulation and pass it on to the hardware interfaces.
   *
   * This will e.g. read the joint positions, velocities and efforts and write them out to
   * controllers via the
   [JointStateInterface](http://docs.ros.org/en/jade/api/hardware_interface/html/c++/classhardware__interface_1_1JointStateInterface.html)
   and/or `franka_hw::FrankaStateInterface`

   *
   * @param[in] time   the current (simulated) ROS time
   * @param[in] period the time step at which the simulation is running
   */
  void readSim(ros::Time time, ros::Duration period) override;

  /**
   * Pass the data send from controllers via the hardware interfaces onto the simulation.
   *
   * This will e.g. write the joint commands (torques or forces) to the corresponding joint in
   * Gazebo in each timestep. These commands are usually send via an
   * [EffortJointInterface](http://docs.ros.org/en/jade/api/hardware_interface/html/c++/classhardware__interface_1_1EffortJointInterface.html)
   *
   * @param[in] time   the current (simulated) ROS time
   * @param[in] period the time step at which the simulation is running
   */
  void writeSim(ros::Time time, ros::Duration period) override;

  /**
   * Set the emergency stop state (not yet implemented)
   *
   * @param[in] active does currently nothing
   */
  void eStopActive(const bool active) override;

  /**
   * Switches the control mode of the robot arm
   * @param start_list list of controllers to start
   * @param stop_list list of controllers to stop
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /**
   * Check (in non-realtime) if given controllers could be started and stopped from the current
   * state of the RobotHW with regard to necessary hardware interface switches and prepare the
   * switching. Start and stop list are disjoint. This handles the check and preparation, the actual
   * switch is commited in doSwitch().
   */
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& /*stop_list*/) override;

 private:
  /// If gazebo::Joint::GetForceTorque() yielded already a non-zero value
  bool robot_initialized_;

  std::unique_ptr<ControllerVerifier> verifier_;

  std::array<double, 3> gravity_earth_;

  std::string arm_id_;
  gazebo::physics::ModelPtr robot_;
  std::map<std::string, std::shared_ptr<franka_gazebo::Joint>> joints_;

  hardware_interface::JointStateInterface jsi_;
  hardware_interface::EffortJointInterface eji_;
  hardware_interface::PositionJointInterface pji_;
  hardware_interface::VelocityJointInterface vji_;
  franka_hw::FrankaStateInterface fsi_;
  franka_hw::FrankaModelInterface fmi_;

  boost::sml::sm<franka_gazebo::StateMachine, boost::sml::thread_safe<std::mutex>> sm_;
  franka::RobotState robot_state_;
  std::unique_ptr<franka_hw::ModelBase> model_;

  double tau_ext_lowpass_filter_;

  ros::Publisher robot_initialized_pub_;
  ros::ServiceServer service_set_ee_;
  ros::ServiceServer service_set_k_;
  ros::ServiceServer service_set_load_;
  ros::ServiceServer service_collision_behavior_;
  ros::ServiceServer service_user_stop_;
  ros::ServiceClient service_gazebo_set_model_configuration_;
  ros::ServiceServer service_set_model_configuration_;
  ros::ServiceClient service_controller_list_;
  ros::ServiceClient service_controller_switch_;
  std::unique_ptr<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>> action_recovery_;

  std::vector<double> lower_force_thresholds_nominal_;
  std::vector<double> upper_force_thresholds_nominal_;

  void initJointStateHandle(const std::shared_ptr<franka_gazebo::Joint>& joint);
  void initEffortCommandHandle(const std::shared_ptr<franka_gazebo::Joint>& joint);
  void initPositionCommandHandle(const std::shared_ptr<franka_gazebo::Joint>& joint);
  void initVelocityCommandHandle(const std::shared_ptr<franka_gazebo::Joint>& joint);
  void initFrankaStateHandle(const std::string& robot,
                             const urdf::Model& urdf,
                             const transmission_interface::TransmissionInfo& transmission);
  void initFrankaModelHandle(const std::string& robot,
                             const urdf::Model& urdf,
                             const transmission_interface::TransmissionInfo& transmission,
                             double singularity_threshold);
  void initServices(ros::NodeHandle& nh);

  void restartControllers();

  void updateRobotState(ros::Time time);
  void updateRobotStateDynamics();

  bool readParameters(const ros::NodeHandle& nh, const urdf::Model& urdf);

  void guessEndEffector(const ros::NodeHandle& nh, const urdf::Model& urdf);

  static double positionControl(Joint& joint, double setpoint, const ros::Duration& period);
  static double velocityControl(Joint& joint, double setpoint, const ros::Duration& period);

  template <int N>
  std::array<double, N> readArray(std::string param, std::string name = "") {
    std::array<double, N> x;

    std::istringstream iss(param);
    std::vector<std::string> values{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    if (values.size() != N) {
      throw std::invalid_argument("Expected parameter '" + name + "' to have exactely " +
                                  std::to_string(N) + " numbers separated by spaces, but found " +
                                  std::to_string(values.size()));
    }
    std::transform(values.begin(), values.end(), x.begin(),
                   [](std::string v) -> double { return std::stod(v); });
    return x;
  }

  /**
   * Helper function for generating a skew symmetric matrix for a given input vector such  that:
   * \f$\mathbf{0} = \mathbf{M} \cdot \mathrm{vec}\f$
   *
   * @param[in] vec the 3D input vector for which to generate the matrix for
   * @return\f$\mathbf{M}\f$ i.e. a skew symmetric matrix for `vec`
   */
  static Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d vec_hat;
    // clang-format off
    vec_hat <<
              0, -vec(2),  vec(1),
         vec(2),      0, -vec(0),
        -vec(1),  vec(0),       0;
    // clang-format on
    return vec_hat;
  }

  /**
   * Shift the moment of inertia tensor by a given offset.
   *
   * This method is based on Steiner's [Parallel Axis
   * Theorem](https://de.wikipedia.org/wiki/Steinerscher_Satz#Verallgemeinerung_auf_Tr%C3%A4gheitstensoren)
   *
   * \f$\mathbf{I^{(p)}} = \mathbf{I} + m \tilde{p}^\top \tilde{p}\f$
   *
   * where \f$\tilde{p}\f$ is the @ref skewMatrix of `p`
   *
   * @param[in] I the inertia tensor defined in the original frame or center or mass of `m`
   * @param[in] m the mass of the body in \f$kg\f$
   * @param[in] p the offset vector to move the inertia tensor along starting from center of mass
   * @return the shifted inertia tensor \f$\mathbf{I^{\left( p \right)}}\f$
   */
  static Eigen::Matrix3d shiftInertiaTensor(Eigen::Matrix3d I, double m, Eigen::Vector3d p) {
    Eigen::Matrix3d P = skewMatrix(p);
    Eigen::Matrix3d Ip = I + m * P.transpose() * P;
    return Ip;
  }

  void forControlledJoint(
      const std::list<hardware_interface::ControllerInfo>& controllers,
      const std::function<void(franka_gazebo::Joint& joint, const ControlMethod&)>& f);
};

}  // namespace franka_gazebo
