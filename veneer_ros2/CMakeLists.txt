cmake_minimum_required(VERSION 3.8)
project(veneer_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(veneer REQUIRED)

find_package(rclcpp REQUIRED)

add_library(ros2_logger STATIC
  src/veneer_ros2_logger/src/veneer_ros2_logger.cpp
)
target_include_directories(ros2_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/veneer_ros2_logger/include>
  $<INSTALL_INTERFACE:include/veneer_ros2_logger>
)
set_property(TARGET ros2_logger PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ros2_logger PUBLIC veneer::veneer rclcpp::rclcpp)

ament_export_libraries(
  ros2_logger
)

install(
  TARGETS ros2_logger
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY src/veneer_ros2_logger/include/
  DESTINATION include/veneer_ros2_logger/
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(veneer)

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
