cmake_minimum_required(VERSION 3.8)
project(monkey_brain_ros_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(monkey_brain_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(veneer_ros2 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  INTERFACE
    rclcpp::rclcpp
)

add_library(ros_context src/ros_context/src/ros_context.cpp)
target_include_directories(ros_context PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ros_context/include>
  $<INSTALL_INTERFACE:include/ros_context>
)
target_link_libraries(ros_context PUBLIC
  monkey_brain_core::context rclcpp::rclcpp PRIVATE veneer_ros2::ros2_logger)
add_library(monkey_brain_ros_utils::ros_context ALIAS ros_context)


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

install(TARGETS ${PROJECT_NAME} ros_context
  EXPORT "export_${PROJECT_NAME}"
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  DIRECTORY src/ros_context/include/
  DESTINATION include/ros_context/
)

ament_export_dependencies(monkey_brain_core rclcpp veneer_ros2)
ament_export_targets("export_${PROJECT_NAME}")

ament_package()
