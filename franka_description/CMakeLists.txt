cmake_minimum_required(VERSION 3.10)
project(franka_description)

find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS xacro)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/franka_robot_urdf.py)
  catkin_add_nosetests(test/hand_urdf.py)
  catkin_add_nosetests(test/dual_panda_example_urdf.py)
endif()
