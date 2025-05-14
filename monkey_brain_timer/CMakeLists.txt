cmake_minimum_required(VERSION 3.8)
project(monkey_brain_timer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(monkey_brain_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(timer_io_plugin SHARED src/timer_io_plugin.cpp)
target_link_libraries(timer_io_plugin PRIVATE pluginlib::pluginlib
        monkey_brain_core::io_plugin rclcpp::rclcpp)
target_include_directories(timer_io_plugin PRIVATE
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/exports/>
)
set_target_properties(timer_io_plugin PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN 1
)

add_library(oneshot_timer_io_plugin SHARED src/oneshot_timer_io_plugin.cpp)
target_link_libraries(oneshot_timer_io_plugin PRIVATE pluginlib::pluginlib
        monkey_brain_core::io_plugin rclcpp::rclcpp)
set_target_properties(oneshot_timer_io_plugin PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN 1
)

pluginlib_export_plugin_description_file(monkey_brain timer_io_plugin.xml)
pluginlib_export_plugin_description_file(monkey_brain oneshot_timer_io_plugin.xml)

ament_export_libraries(
  timer_io_plugin
  oneshot_timer_io_plugin
)

install(
  TARGETS timer_io_plugin oneshot_timer_io_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(
  export_${PROJECT_NAME}
)

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
