cmake_minimum_required(VERSION 3.22)
project(dofbot_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(dofbot_ros2_control SHARED
  src/dofbot_topic_hardware.cpp
)

target_include_directories(dofbot_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(dofbot_ros2_control PUBLIC cxx_std_17)

ament_target_dependencies(dofbot_ros2_control
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface dofbot_ros2_control_plugins.xml)

install(TARGETS dofbot_ros2_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES dofbot_ros2_control_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib sensor_msgs)

ament_package()
