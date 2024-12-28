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
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


add_message_files(
  FILES
  Position.msg
)

add_service_files(
  FILES
  JointMotion.srv
)

generate_messages(DEPENDENCIES std_msgs)


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
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(multi_agent_vector_fields
  src/cf_manager.cpp
  src/cf_agent.cpp
)

target_link_libraries(multi_agent_vector_fields
  yaml-cpp
  ${catkin_LIBRARIES}
)


add_executable(main_node src/main.cpp)


target_link_libraries(main_node
  multi_agent_vector_fields
  yaml-cpp
  ${catkin_LIBRARIES}
)


install(TARGETS multi_agent_vector_fields main_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

