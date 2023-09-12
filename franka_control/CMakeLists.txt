cmake_minimum_required(VERSION 3.4)
project(franka_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  franka_hw
  franka_msgs
  geometry_msgs
  message_generation
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  tf
  tf2_msgs
  std_srvs
)

find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

# merge Franka + catkin INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_state_controller
  CATKIN_DEPENDS
    controller_interface
    franka_hw
    franka_msgs
    geometry_msgs
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
    tf2_msgs
    std_srvs
  DEPENDS Franka
)

## franka_state_controller
add_library(franka_state_controller
  src/franka_state_controller.cpp
)
if (Franka_VERSION GREATER_EQUAL 0.9)
    target_compile_definitions(franka_state_controller PUBLIC ENABLE_BASE_ACCELERATION)
endif()
add_dependencies(franka_state_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_state_controller
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_state_controller SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_state_controller PUBLIC
  include
)

## franka_control_node
add_executable(franka_control_node
  src/franka_control_node.cpp
)

add_dependencies(franka_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_control_node
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_control_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_executable(franka_combined_control_node
    src/franka_combined_control_node.cpp
)

add_dependencies(franka_combined_control_node
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_combined_control_node
  ${catkin_LIBRARIES}
)

target_include_directories(franka_combined_control_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)


## Installation
install(TARGETS franka_state_controller
                franka_control_node
                franka_combined_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
  add_format_target(franka_control FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_control
    FILES ${SOURCES}
    DEPENDS franka_control_node franka_combined_control_node franka_state_controller
  )
endif()
