cmake_minimum_required(VERSION 3.8)
project(monkey_brain_yasmin_state_observer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(monkey_brain_core REQUIRED)
find_package(monkey_brain_scxml REQUIRED)
find_package(monkey_brain_ros_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin_msgs REQUIRED)

add_library(yasmin_state_observer SHARED src/yasmin_state_observer.cpp)
target_include_directories(yasmin_state_observer PUBLIC include)
target_link_libraries(yasmin_state_observer PUBLIC
  monkey_brain_core::context
  monkey_brain_scxml::state_machine_observer
  monkey_brain_ros_utils::ros_context
  pluginlib::pluginlib
  ${yasmin_msgs_TARGETS}
  rclcpp::rclcpp
)
pluginlib_export_plugin_description_file(monkey_brain sm_observer_plugin.xml)

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

  ament_add_gmock(test_yasmin_state_observer test/test_yasmin_state_observer.cpp)
  target_link_libraries(test_yasmin_state_observer yasmin_state_observer monkey_brain_scxml::test_utils)
endif()

install(TARGETS yasmin_state_observer DESTINATION lib/${PROJECT_NAME})

ament_package()
