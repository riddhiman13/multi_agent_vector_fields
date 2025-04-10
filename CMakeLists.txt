cmake_minimum_required(VERSION 3.0.2)
project(multi_agent_vector_fields)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  eigen_conversions
  roslib
  moveit_msgs
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_agent_vector_fields
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    moveit_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# multi_agent_vector_fields library
add_library(multi_agent_vector_fields
  src/cf_manager.cpp
  src/cf_agent.cpp
  src/visualize_helper.cpp
)
target_link_libraries(multi_agent_vector_fields PUBLIC
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# different node libraries
add_executable(main_node src/agent_plus_scene.cpp)
target_link_libraries(main_node PUBLIC
  multi_agent_vector_fields
  yaml-cpp
)

add_executable(test_node src/agent_plus_scene_status)
target_link_libraries(test_node PUBLIC
  multi_agent_vector_fields
  yaml-cpp
)

add_executable(multi_agent_feedback_node src/agent_plus_scene_status_new.cpp)
target_link_libraries(multi_agent_feedback_node PUBLIC
  multi_agent_vector_fields
  yaml-cpp
)


add_executable(bench_node src/planner_benchmark.cpp)
target_link_libraries(bench_node
  multi_agent_vector_fields
  yaml-cpp
)