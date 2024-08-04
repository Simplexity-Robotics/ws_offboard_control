cmake_minimum_required(VERSION 3.5)
project(image_processing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(image_processing_node src/image_processing_node.cpp)

ament_target_dependencies(image_processing_node
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  cv_bridge
)

target_compile_features(image_processing_node PRIVATE cxx_std_17)

install(TARGETS
  image_processing_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
