cmake_minimum_required(VERSION 3.15)
project(sbmpo_ros2)

# Set C++ standard library
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use Release mode
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sbmpo REQUIRED)
find_package(sbmpo_msgs REQUIRED)

# Specify the directory containing header files for your library
include_directories(include)

include_directories(
  ${sbmpo_msgs_INCLUDE_DIRS}
)

# Create a shared library target
add_library(sbmpo_ros2 SHARED
  src/unicycle_steering_action_server.cpp
  src/unicycle_steering_action_client.cpp
)

# Set the include directories for the library target
# Set include directories for library target
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link the necessary dependencies to the library target
ament_target_dependencies(sbmpo_ros2 
  rclcpp rclcpp_action rclcpp_components sbmpo sbmpo_msgs)

# Register example action server
rclcpp_components_register_node(sbmpo_ros2 
  PLUGIN "sbmpo_ros2::UnicycleSteeringActionServer"
  EXECUTABLE unicycle_steering_action_server)

# Register example action client
rclcpp_components_register_node(sbmpo_ros2 
PLUGIN "sbmpo_ros2::UnicycleSteeringActionClient"
EXECUTABLE unicycle_steering_action_client)

# Install the created library and its headers
install(
  TARGETS sbmpo_ros2
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export the library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action rclcpp_components sbmpo sbmpo_msgs)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_package()
