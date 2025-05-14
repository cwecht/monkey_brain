cmake_minimum_required(VERSION 3.8)
project(veneer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(veneer INTERFACE)
target_include_directories(veneer INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/veneer/include>
  $<INSTALL_INTERFACE:include/veneer>
)

add_library(stdout_logger STATIC
  src/veneer_stdout_logger/src/veneer_stdout_logger.cpp
)
target_include_directories(stdout_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/veneer_stdout_logger/include>
  $<INSTALL_INTERFACE:include/veneer_stdout_logger>
)
set_property(TARGET stdout_logger PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(stdout_logger PUBLIC veneer)

add_library(veneer_gmock INTERFACE)
target_include_directories(veneer_gmock INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/veneer_gmock/include>
  $<INSTALL_INTERFACE:include/veneer_gmock>
)
target_link_libraries(veneer_gmock INTERFACE veneer)

install(
  TARGETS veneer stdout_logger veneer_gmock
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY src/veneer/include/
  DESTINATION include/veneer/
)

install(
  DIRECTORY src/veneer_stdout_logger/include/
  DESTINATION include/veneer_stdout_logger/
)

install(
  DIRECTORY src/veneer_gmock/include/
  DESTINATION include/veneer_gmock/
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(stdout_logger)

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
