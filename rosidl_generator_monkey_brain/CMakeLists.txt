cmake_minimum_required(VERSION 3.12)

project(rosidl_generator_monkey_brain)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_index_register_resource("rosidl_generator_packages")

ament_python_install_package(${PROJECT_NAME})

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

ament_package(
  CONFIG_EXTRAS "rosidl_generator_monkey_brain-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_monkey_brain
  DESTINATION lib/rosidl_generator_monkey_brain
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
