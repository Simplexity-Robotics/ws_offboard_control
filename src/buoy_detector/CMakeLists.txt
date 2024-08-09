cmake_minimum_required(VERSION 3.5)
project(buoy_detector)  # This should match the package name in package.xml

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(buoy_detector src/buoy_detector.cpp)

# Link dependencies
ament_target_dependencies(buoy_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  buoy_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
