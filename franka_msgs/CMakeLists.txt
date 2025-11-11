cmake_minimum_required(VERSION 3.10)
project(franka_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs actionlib_msgs)

add_message_files(FILES Errors.msg FrankaState.msg)

add_service_files(FILES
  SetCartesianImpedance.srv
  SetEEFrame.srv
  SetForceTorqueCollisionBehavior.srv
  SetFullCollisionBehavior.srv
  SetJointConfiguration.srv
  SetJointImpedance.srv
  SetKFrame.srv
  SetLoad.srv
)

add_action_files(FILES ErrorRecovery.action)

generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs actionlib_msgs)
