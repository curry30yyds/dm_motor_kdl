cmake_minimum_required(VERSION 3.0.2)
project(dm_motor_controller)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  yaml-cpp
)

find_package(yaml-cpp REQUIRED) 
find_package(orocos_kdl REQUIRED)

add_message_files(
  FILES
  MITCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)


add_executable(dm_motor_node
  src/dm_motor_node.cpp
)

add_dependencies(dm_motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dm_motor_node
  ${catkin_LIBRARIES}
)


add_executable(publish_trajectory
  src/publish_trajectory.cpp
)

add_dependencies(publish_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(publish_trajectory
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  yaml-cpp
)

add_executable(kdl_trajectory_generator src/kdl_trajectory_generator.cpp)
add_dependencies(kdl_trajectory_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kdl_trajectory_generator
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  yaml-cpp
)

add_executable(trajectory_mit_node src/trajectory_mit_node.cpp)
target_link_libraries(trajectory_mit_node
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  yaml-cpp
)

add_executable(motor_zeroing_node src/motor_zeroing_node.cpp)
target_link_libraries(motor_zeroing_node ${catkin_LIBRARIES})