cmake_minimum_required(VERSION 3.10)
project(franka_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  controller_interface
  combined_robot_hw
  hardware_interface
  joint_limits_interface
  roscpp
  std_srvs
  pluginlib
  urdf
  franka_msgs
  message_generation
)

find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# merge Franka + catkin INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_hw franka_control_services
  CATKIN_DEPENDS
    actionlib_msgs
    actionlib
    controller_interface
    combined_robot_hw
    hardware_interface
    joint_limits_interface
    roscpp
    std_srvs
    pluginlib
    urdf
    franka_msgs
  DEPENDS Franka
)

add_library(franka_hw
  src/control_mode.cpp
  src/franka_hw.cpp
  src/franka_combinable_hw.cpp
  src/franka_combined_hw.cpp
  src/resource_helpers.cpp
  src/trigger_rate.cpp
)

add_dependencies(franka_hw
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_hw
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  franka_control_services
)

target_include_directories(franka_hw SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_hw PUBLIC
  include
)

if (Franka_VERSION GREATER_EQUAL 0.9)
  target_compile_definitions(franka_hw PUBLIC ENABLE_BASE_ACCELERATION)
endif()

## franka_control_services
add_library(franka_control_services
  src/services.cpp
)

add_dependencies(franka_control_services
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_control_services
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_control_services SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(franka_control_services PUBLIC
  include
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

## Installation
install(TARGETS franka_hw franka_control_services
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_hw FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_hw
    FILES ${SOURCES}
    DEPENDS franka_hw
  )
endif()
