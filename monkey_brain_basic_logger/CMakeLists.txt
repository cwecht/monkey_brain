cmake_minimum_required(VERSION 3.8)
project(monkey_brain_basic_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wconversion)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(monkey_brain_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(string_view_split INTERFACE)
target_include_directories(string_view_split INTERFACE src/string_view_split/include)

add_library(basic_logger STATIC src/basic_logger/src/basic_logger.cpp)
target_include_directories(basic_logger PUBLIC src/basic_logger/include)
target_link_libraries(basic_logger PUBLIC monkey_brain_core::io_plugin PRIVATE
        string_view_split)
set_property(TARGET basic_logger PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(basic_logger PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN 1
)

add_library(basic_logger_io_plugin SHARED src/basic_logger_io_plugin.cpp)
target_link_libraries(basic_logger_io_plugin
        PRIVATE
          basic_logger
        PUBLIC
          rclcpp::rclcpp
          pluginlib::pluginlib
          monkey_brain_core::io_plugin)
set_target_properties(basic_logger_io_plugin PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN 1
)

pluginlib_export_plugin_description_file(monkey_brain io_plugin.xml)

ament_export_libraries(
  basic_logger_io_plugin
)

install(
  TARGETS basic_logger_io_plugin
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

  ament_add_gmock(test_split test/test_split.cpp)
  target_link_libraries(test_split string_view_split)

  ament_add_gmock(test_basic_logger test/test_basic_logger.cpp)
  target_link_libraries(test_basic_logger basic_logger)
endif()

ament_package()
