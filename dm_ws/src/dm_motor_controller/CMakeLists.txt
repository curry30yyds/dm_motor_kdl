cmake_minimum_required(VERSION 3.0.2)
project(dm_motor_controller)

add_compile_options(-std=c++14)

# 1. 查找 catkin 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  std_srvs
  hy_hardware_interface
)

# 2. 查找 yaml-cpp 和 orocos_kdl
find_package(yaml-cpp REQUIRED)
find_package(orocos_kdl REQUIRED)

# 3. 添加消息文件
add_message_files(
  FILES
  mit_control_frame.msg
)

# 4. 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 5. 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime hy_hardware_interface
)

# 6. 包含头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# 7. 各节点 target

add_executable(dm_motor_node src/dm_motor_node.cpp)
add_dependencies(dm_motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dm_motor_node ${catkin_LIBRARIES})

add_executable(publish_trajectory src/publish_trajectory.cpp)
add_dependencies(publish_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_trajectory ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} yaml-cpp)

add_executable(kdl_trajectory_generator src/kdl_trajectory_generator.cpp)
add_dependencies(kdl_trajectory_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kdl_trajectory_generator ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} yaml-cpp)

add_executable(trajectory_mit_node src/trajectory_mit_node.cpp)
add_dependencies(trajectory_mit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_mit_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} yaml-cpp)

add_executable(motor_zeroing_node src/motor_zeroing_node.cpp)
add_dependencies(motor_zeroing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_zeroing_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} yaml-cpp)

add_executable(trajectory_comparison_node src/trajectory_comparison_node.cpp)
add_dependencies(trajectory_comparison_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_comparison_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} yaml-cpp)

add_executable(dm_4dof_control_hwi_node src/dm_4dof_control_hwi.cpp)
add_dependencies(dm_4dof_control_hwi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dm_4dof_control_hwi_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} yaml-cpp)

add_executable(test_joint_and_position_publisher src/test/test_publisher.cpp)
add_dependencies(test_joint_and_position_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_joint_and_position_publisher ${catkin_LIBRARIES})


