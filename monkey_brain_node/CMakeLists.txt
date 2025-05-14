cmake_minimum_required(VERSION 3.8)
project(monkey_brain_node)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(monkey_brain_core REQUIRED)
find_package(monkey_brain_ros_utils REQUIRED)
#find_package(veneer REQUIRED)

add_library(plugins_parser src/plugins_parser/src/plugins_parser.cpp)
target_include_directories(plugins_parser PUBLIC src/plugins_parser/include)
target_link_libraries(plugins_parser PUBLIC yaml-cpp)

add_library(plugins_loader src/plugins_loader/src/plugins_loader.cpp)
target_include_directories(plugins_loader PUBLIC src/plugins_loader/include)
target_link_libraries(plugins_loader
  PUBLIC
    plugins_parser
    monkey_brain_core::context
    monkey_brain_core::decision_engine
    monkey_brain_core::io_plugin_registry
    monkey_brain_core::operations_factory
    pluginlib::pluginlib
    rclcpp::rclcpp
)

add_library(node src/node/src/monkey_brain_node.cpp)
target_include_directories(node PUBLIC src/node/include)
target_link_libraries(node
  plugins_parser
  plugins_loader
  monkey_brain_core::context
  monkey_brain_core::decision_engine
  monkey_brain_core::io_plugin_registry
  monkey_brain_core::concrete_operations_factory
  monkey_brain_ros_utils::ros_context
  pluginlib::pluginlib
  rclcpp::rclcpp
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    node
  PUBLIC
    ${std_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
)

add_executable(monkey_tool tools/monkey_tool.cpp)
target_link_libraries(monkey_tool
  plugins_parser
  plugins_loader
  pluginlib::pluginlib
  monkey_brain_core::cli_context
  monkey_brain_core::concrete_operations_factory
  monkey_brain_core::io_plugin
  monkey_brain_core::io_plugin_registry
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS monkey_tool DESTINATION)

ament_environment_hooks("tools/monkey_tool_complete.bash")

ament_package()
