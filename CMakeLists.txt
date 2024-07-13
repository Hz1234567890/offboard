cmake_minimum_required(VERSION 3.8)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(OpenCV 4.7.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
add_executable(offboard src/main.cpp src/OffboardControl.cpp src/OffboardControl.hpp src/init.cpp src/speed.cpp src/takeoff.cpp src/module.cpp src/YOLO.hpp src/PID.cpp)
add_executable(only src/offboard.cpp)
ament_target_dependencies(offboard rclcpp geometry_msgs mavros_msgs OpenCV cv_bridge ros2_interfaces sensor_msgs)
ament_target_dependencies(only rclcpp geometry_msgs mavros_msgs)

install(TARGETS
  offboard
  only
  DESTINATION lib/${PROJECT_NAME}
)
