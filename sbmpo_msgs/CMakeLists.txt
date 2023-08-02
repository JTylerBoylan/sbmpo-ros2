cmake_minimum_required(VERSION 3.8)
project(sbmpo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# Make ROS custom msg files
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(sbmpo_msgs
  "msg/State.msg"
  "msg/Control.msg"
  "msg/SearchParameters.msg"
  "msg/SearchResults.msg"
  "action/RunSBMPO.action"
  DEPENDENCIES std_msgs
)

ament_package()
