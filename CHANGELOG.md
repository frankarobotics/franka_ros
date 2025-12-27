# CHANGELOG

## 0.10.3 - 2025-10-20

Requires `libfranka` >= 0.8.0

* docs: Move documentation under each package.

## 0.10.2 - 2023-12-20

Requires `libfranka` >= 0.8.0

  * `franka_gazebo`: Offer both `panda.launch` and `fr3.launch`.
  * `franka_gazebo`: Fix velocity control by adding the missing effort.
  * `franka_control`: Clear the error flag if the robot is in `kIdle` mode, i.e. ready to move.
  * `franka_control`: Fix unexpected long delay when calling error recovery ([#317](https://github.com/frankaemika/franka_ros/issues/317))
  * Fix a possible compilation error by sorting include directories by topological order ([#319](https://github.com/frankaemika/franka_ros/issues/319)).
  * `franka_control`: Fix a bug where `error_recovery` actions recover future errors ([#316](https://github.com/frankaemika/franka_ros/issues/316)).
  * `franka_gazebo`: `FrankaHWSim` only acts on joints belonging to a Franka robot. This allows to combine a Franka robot and others (like mobile platforms) in same URDF ([#313](https://github.com/frankaemika/franka_ros/issues/313))
  * `franka_description`: `<xacro:franka_robot/>` macro now supports to customize the `parent` frame and its `xyz` + `rpy` offset
  * `franka_hw`: Fix the bug where the previous controller is still running after switching the controller. ([#326](https://github.com/frankaemika/franka_ros/issues/326))
  * `franka_gazebo`: Add `set_franka_model_configuration` service.
  * `franka_gazebo`: Fix external torque calculation by accounting for internal joint limits

## 0.10.1 - 2022-09-15

Requires `libfranka` >= 0.8.0

  * `franka_example_controllers`: Fix examples not working on melodic due to non-ascii symbols in comments of `franka_example_controllers.yaml`.
  * `franka_example_controllers`: The `link_name` in  `cartesian_impedance_example_controller.launch` is now determined over the `arm_id`.
  * `franka_description`: Fix catkin warning about `catkin_add_nosetests`

## 0.10.0 - 2022-09-02

Requires `libfranka` >= 0.8.0

  * `franka_example_controllers`: Normalize rotations before usage
  * `franka_example_controllers`: Extend the `teleop_joint_pd_example_controller` with markers indicating leader and follower roles + consistently use leader and follower as robot names in the example.
  * `franka_example_controllers`: Don't require MoveIt for `move_to_start.launch`
  * **BREAKING** `franka_description`: Refactor URDF files to also support FR3
  * `franka_control`: Introduce optional `robot` argument in the `franka_control.launch` launch file that chooses either the URDF for panda or fr3. This argument can also be used in the launch files of `franka_examples`. The default is "panda".

## 0.9.1 - 2022-08-29

Requires `libfranka` >= 0.8.0

  * `franka_example_controllers`: Extend the `teleop_joint_pd_example_controller` with markers indicating leader and follower roles + consistently use leader and follower as robot names in the example.
  * `franka_example_controllers`: Extend the `teleop_joint_pd_example_controller` with a finite state machine that aligns the follower robot before starting to track the leader.
  * `franka_example_controllers`: Extend the `teleop_joint_pd_example_controller` with joint walls to actively avoid position or velocity limit violations.
  * `franka_example_controllers`: Fix namespacing of dynamic reconfigure node of cartesian impedance example controller
  * `franka_control`: Configurable `arm_id` in launch & config files
  * `franka_description`: URDF now contains `$(arm_id)_linkN_sc` links containing the capsule collision modules used for self-collision avoidance (MoveIt).
  * `franka_description`: Unit test suite for URDFs
  * `franka_description`: Make `util.xacro` be includable from other packages
  * `franka_description`: Add `tcp_xyz` & `tcp_rpy` offsets to specify a custom TCP for MoveIT's `$(arm_id)_manipulator` move group
  * `franka_control`: `franka_control.launch` accepts `xacro_args` to pass down additional arguments to URDF
  * `franka_gazebo`: `panda.launch` accepts `xacro_args` to pass down additional arguments to URDF
  * `franka_gazebo`: Fix motion generator config respects `arm_id`
  *  **BREAKING**: `gripper_action` goes now to the commanded gripper position when `max_effort` is zero
  * `franka_gazebo`: Drop `delayed_controller_spawner.py` script in favor of `--wait-for TOPIC` flag from controller_manager
  * `franka_gazebo`: Properly calculate inertial properties of `world/stone/model.sdf`
  * `franka_gazebo`: `set_user_stop` service to simulate User stop in Gazebo
  * `franka_gazebo`: `error_recovery` action similar to `franka_control`
  * **BREAKING**: `franka_gazebo`: Move services like `set_EE_frame`, `set_K_frame` ... into `franka_control` namespace to be more consistent with real robot

## 0.9.0 - 2022-03-29

Requires `libfranka` >= 0.8.0

  * Added support for libfranka 0.9.0
  * **BREAKING** add base acceleration `O_ddP_O` (will for now always be {0,0,-9.81}) to `FrankaState.msg`
  * **BREAKING** add  following errors to `Errors.msg`:
    * `joint_move_in_wrong_direction`
    * `cartesian_spline_motion_generator_violation`
    * `joint_via_motion_generator_planning_joint_limit_violation`
    * `base_acceleration_initialization_timeout`
    * `base_acceleration_invalid_reading`
  * `franka_gazebo`:
    - Add JointPosition and JointVelocity Interface
    - Fix: Robot now keeps position when no controller is running
    - joint_{position,velocity}_example controller are now available in `franka_gazebo`

## 0.8.2 - 2022-02-22

Requires `libfranka` >= 0.8.0

  * `franka_gazebo`:
    - Fix: homing action works again
    - Fix: move action can fail instead of blocking indefinitely
    - Fix: align behavior of grasping action with `franka_gripper`
    - Add `joint_state_desired` publisher
    - Add singularity warning  if Jacobian becomes singular
    - Make `/panda` namespace optional
    - Make finger collisions primitive in `franka_gazebo`
    - Add 'gravity_vector' gravity ROS parameter to FrankaHWSim
    - Improve Gazebo 'stone' world objects
    - Introduce new `tau_ext_lowpass_filter` parameter for `franka_gazebo` to configure the filtering of `tau_ext_hat_filtered`
    - Add realistic hand/finger collision geometries to the Gazebo robot description
  * Fix: Allow interactive marker server to shut down if not initialized
  * No further ROS Kinetic support, since [End-of-Life was in April 2021](http://wiki.ros.org/Distributions)
  * Make position + orientation targets threadsafe in cartesian example controller
  * Add effort joint trajectory controller to be used by MoveIT
  * Fix "Failed to create robot simulation interface loader" bug when installing from APT
  * Add `connected_to` option to `panda_gazebo.xacro` macro, similar to `panda_arm.xacro`
  * Rename `ns` -> `arm_id` in `hand.xacro` macros to be consistent with the other xacro files

## 0.8.1 - 2021-09-08

Requires `libfranka` >= 0.8.0

  * `franka_hw`:
    - Add `bool hasError()` member function to `FrankaCombinableHW`
    - Only execute controller update in `franka_combined_control_node` when not in error state
    - Reset command buffer upon recover in `FrankaCombinableHW`

## 0.8.0 - 2021-08-03

Requires `libfranka` >= 0.8.0

  * `franka_hw`, `franka_combinable_hw`, `franka_combined_hw`: Added service interface to disconnect
    and reconnect when no controller is active. This allows mixing FCI- and DESK-based application
    without stopping the according hardware nodes.
  * **BREAKING** `franka_hw`, `franka_combinable_hw` method control() now is non-const to allow
    locking a mutex member variable.
  * **BREAKING** Change behavior of `franka_msgs/SetEEFrame`. Previously, this method would set
    the flange-to-end-effector transformation `F_T_EE`. This has been split up into two transformations:
    `F_T_NE`, only settable in Desk, and `NE_T_EE`, which can be set in `franka_ros` with `SetEEFrame`
    and defaults to the identity transformation.
  * Add `F_T_NE` and `NE_T_EE` to `franka_msgs/FrankaState`.
  * _Franka Gazebo Integration_: Now you can simulate Panda robots in Gazebo including:
    * gravity compensation
    * non-realtime commands like `setEEFrame` or `setLoad`
    * gripper simulation with the _same_ action interface as `franka_gripper`
    * estimated inertias in the URDF
    * no need to change existing ROS controllers
    * only torque control supported in this version
  * Extract Model Library in abstract base class interface. This allows users to implement their own model.
  * **BREAKING** Remove `panda_arm_hand.urdf.xacro`. Use `panda_arm.urdf.xacro hand:=true` instead.

## 0.7.1 - 2020-10-22

Requires `libfranka` >= 0.7.0

  * `franka_example_controllers`: Added example for dual-arm teleoperation based on `franka_combinable_hw`.
  * `franka_gripper`: Made stopping on shutdown optional.
  * `franka_hw`: Added `franka_control_services` install instruction.

## 0.7.0 - 2020-07-15

Requires `libfranka` >= 0.7.0

  * **BREAKING** moved services and action from `franka_control` to `franka_msgs`.
  * **BREAKING** moved Service container from `franka_control` to `franka_hw`.
  * `franka_example_controllers`: Added example for dual-arm control based on `franka_combinable_hw`.
  * `franka_description` :
    - Added an example urdf with two panda arms.
    - **BREAKING** Updated collision volumes.
    - Removed invalid `axis` for `joint8`.
  * `franka_hw`:
    - Added hardware classes to support torque-controlling multiple robots from one controller.
    - Refactored FrankaHW class to serve as base class (e.g. for FrankaCombinableHW).
    - Added joint limits checking to FrankaHW which means parameterized warning prints when approaching limits.
    - Made initial collision behavior a parameter.
    - Added default constructor and init method to FrankaHW.
    - **BREAKING** moved parsing of parameters from control node to FrankaHW::init.
    - **BREAKING** made libfranka robot a member of FrankaHW.
    - Added missing return value to `franka::ControllerMode` stream operator function.
  * `franka_control`:
    - Added control node that can runs a `FrankaCombinedHW` to control mulitple Pandas.
    - Publish whole `libfranka` `franka::RobotState` in `franka_state_controller`.
  * `franka_example_controllers`:
    - Cartesian impedance example controller: Interpolate desired orientations with slerp and change orientation error
      to quaternion.
  * **BREAKING** Moved `panda_moveit_config` to [`ros-planning`](https://github.com/ros-planning/panda_moveit_config).
  * Added support for ROS Melodic Morenia.
  * Raised minimum CMake version to 3.4 to match `libfranka`.
  * Add rosparam to choose value of `franka::RealtimeConfig`.
  * Fix unused parameter bugs in `FrankaModelHandle` (#78).
  * Added (experimental) support for ROS Noetic Ninjemys.

## 0.6.0 - 2018-08-08

Requires `libfranka` >= 0.5.0

  * **BREAKING** Fixes for MoveIt, improving robot performance:
    * Fixed joint velocity and acceleration limits in `joint_limits.yaml`
    * Use desired joint state for move group
  * **BREAKING** Updated joint limits in URDF
  * **BREAKING** Fixed velocity, acceleration and jerk limits in `franka_hw`
  * **BREAKING** Start `franka_gripper_node` when giving `load_gripper:=true` to `franka_control.launch`
  * Allow to configure rate limiting, filtering and internal controller in `franka_control_node`
  * **BREAKING** `FrankaHW::FrankaHW` takes additional parameters.
  * **BREAKING** Enabled rate limiting and low-pass filtering by default (`franka_control_node.yaml`)
  * Publish desired joint state in `/joint_state_desired`
  * Removed `effort_joint_trajectory_controller` from `default_controllers.yaml`
  * Fixed a bug when switching between controllers using the same `libfranka` interface

## 0.5.0 - 2018-06-28

Requires `libfranka` >= 0.4.0

  * **BREAKING** Updated URDF:
    * Adjusted maximum joint velocity
    * Updated axis 4 hard and soft limits

## 0.4.1 - 2018-06-21

Requires `libfranka` >= 0.3.0

  * Added some missing includes to `franka_hw`
  * Add support for commanding elbow in Cartesian pose and Cartesian velocity interfaces

## 0.4.0 - 2018-03-26

Requires `libfranka` >= 0.3.0

  * **BREAKING** Removed `arm_id` and default `robot_ip` from launchfiles
  * **BREAKING** Changed namespace of `franka_control` controller manager
  * **BREAKING** Changed behavior of `gripper_action` for compatibility with MoveIt
  * Changes in `panda_moveit_config`:
    * Updated joint limits from URDF
    * Removed `home` poses
    * Fixed fake execution
    * Add `load_gripper` argument (default: `true`) to `panda_moveit.launch`
    * Conditionally load controllers/SRDFs based on `load_gripper`
    * Add gripper controller configuration (requires running `franka_gripper_node`)
  * Added `mimic` tag for gripper fingers to URDF and fixed velocity limits

## 0.3.0 - 2018-02-22

Requires `libfranka` >= 0.3.0

  * **BREAKING** Changed signatures in `franka_hw::FrankaModelHandle`
  * **BREAKING** Added epsilon parameters to `franka_gripper/Grasp` action
  * Added Collada meshes for Panda and Hand
  * Added missing dependencies to `panda_moveit_config` and `franka_example_controllers`
  * Fixed linker errors when building with `-DFranka_DIR` while an older version of
    `ros-kinetic-libfranka` is installed
  * Added gripper joint state publisher to `franka_visualization`
  * Moved `move_to_start.py` example script to `franka_example_controllers`

## 0.2.2 - 2018-01-31

Requires `libfranka` >= 0.2.0

  * Catkin-related fixes for `franka_example_controllers`
  * Added missing `<build_export_depend>` for `message_runtime`

## 0.2.1 - 2018-01-30

Requires `libfranka` >= 0.2.0

  * Added missing dependency to `franka_example_controllers`
  * Lowered rotational gains for Cartesian impedance example controller

## 0.2.0 - 2018-01-29

Requires `libfranka` >= 0.2.0

  * Added missing run-time dependencies to `franka_description` and `franka_control`
  * Added `tau_J_d`, `m_ee`, `F_x_Cee`, `I_ee`, `m_total`, `F_x_Ctotal`, `I_total`,
    `theta` and `dtheta` to `franka_msgs/FrankaState`
  * Added new errors to `franka_msgs/Errors`
  * Updated and improved examples in `franka_example_controllers`
  * Fixed includes for Eigen3 in `franka_example_controllers`
  * Fixed gripper state publishing in `franka_gripper_node`

## 0.1.2 - 2017-10-10

  * Fixed out-of-workspace build

## 0.1.1 - 2017-10-09

  * Integrated `franka_description` as subdirectory
  * Fixed dependencies on libfranka
  * Fixed RViz config file paths
  * Added missing `test_depend` to `franka_hw`
  * Added missing CMake install rules

## 0.1.0 - 2017-09-15

  * Initial release
