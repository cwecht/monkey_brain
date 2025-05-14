cmake_minimum_required(VERSION 3.8)
project(monkey_brain_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wconversion)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(veneer REQUIRED)

add_library(context INTERFACE)
target_include_directories(context INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/context/include>
  $<INSTALL_INTERFACE:include/context>
)
add_library(monkey_brain_core::context ALIAS context)
target_link_libraries(context INTERFACE veneer::veneer)

add_library(cli_context src/cli_context/src/cli_context.cpp)
target_include_directories(cli_context PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cli_context/include>
  $<INSTALL_INTERFACE:include/cli_context>
)
add_library(monkey_brain_core::cli_contextt ALIAS context)
target_link_libraries(cli_context PUBLIC context veneer::stdout_logger)

add_library(io_plugin INTERFACE)
target_include_directories(io_plugin INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/io_plugin/include>
  $<INSTALL_INTERFACE:include/io_plugin>
)
add_library(monkey_brain_core::io_plugin ALIAS io_plugin)
target_link_libraries(io_plugin INTERFACE yaml-cpp)

add_library(expression_description INTERFACE)
target_include_directories(expression_description INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/expression_description/include>
  $<INSTALL_INTERFACE:include/expression_description>
)
add_library(monkey_brain_core::expression_description ALIAS expression_description)

add_library(action_description INTERFACE)
target_include_directories(action_description INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/action_description/include>
  $<INSTALL_INTERFACE:include/action_description>
)
target_link_libraries(action_description INTERFACE monkey_brain_core::expression_description)
add_library(monkey_brain_core::action_description ALIAS action_description)

add_library(action INTERFACE)
target_include_directories(action INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/action/include>
  $<INSTALL_INTERFACE:include/action>
)
add_library(monkey_brain_core::action ALIAS action)

add_library(expression INTERFACE)
target_include_directories(expression INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/expression/include>
  $<INSTALL_INTERFACE:include/expression>
)
add_library(monkey_brain_core::expression ALIAS expression)

add_library(operator_factory INTERFACE)
target_include_directories(operator_factory INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/operator_factory/include>
  $<INSTALL_INTERFACE:include/operator_factory>
)
target_link_libraries(operator_factory
  INTERFACE
    monkey_brain_core::expression
    monkey_brain_core::io_plugin
)
add_library(monkey_brain_core::operator_factory ALIAS operator_factory)

add_library(operations_factory INTERFACE)
target_include_directories(operations_factory INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/operations_factory/include>
  $<INSTALL_INTERFACE:include/operations_factory>
)
target_link_libraries(operations_factory
  INTERFACE
    monkey_brain_core::action
    monkey_brain_core::action_description
    monkey_brain_core::expression
    monkey_brain_core::expression_description
    monkey_brain_core::io_plugin
    monkey_brain_core::operator_factory
)
add_library(monkey_brain_core::operations_factory ALIAS operations_factory)

add_library(environment INTERFACE)
target_include_directories(environment INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/environment/include>
  $<INSTALL_INTERFACE:include/environment>
)
target_link_libraries(environment INTERFACE io_plugin)
add_library(monkey_brain_core::environment ALIAS environment)

add_library(decision_engine INTERFACE)
target_include_directories(decision_engine INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/decision_engine/include>
  $<INSTALL_INTERFACE:include/decision_engine>
)
add_library(monkey_brain_core::decision_engine ALIAS decision_engine)
target_link_libraries(decision_engine INTERFACE environment operations_factory context)

add_library(expression_parser SHARED src/expression_parser/src/parse_expression.cpp)
target_include_directories(expression_parser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/expression_parser/include>
    $<INSTALL_INTERFACE:include/expression_parser>
  PRIVATE
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(expression_parser PUBLIC monkey_brain_core::expression_description)
add_library(monkey_brain_core::expression_parser ALIAS expression_parser)

add_library(action_parser SHARED src/action_parser/src/parse_action.cpp)
target_include_directories(action_parser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/action_parser/include>
    $<INSTALL_INTERFACE:include/action_parser>
  PRIVATE
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(action_parser PUBLIC monkey_brain_core::action_description monkey_brain_core::expression_parser)
add_library(monkey_brain_core::action_parser ALIAS action_parser)

add_library(stl_helper INTERFACE)
target_include_directories(stl_helper INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/stl_helper/include>
  $<INSTALL_INTERFACE:include/stl_helper>
)

add_library(operator_registry STATIC src/operator_registry/src/operator_registry.cpp)
target_include_directories(operator_registry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/operator_registry/include>
  $<INSTALL_INTERFACE:include/operator_registry>
)
target_link_libraries(operator_registry
  PUBLIC
    monkey_brain_core::environment
    monkey_brain_core::expression
    monkey_brain_core::operator_factory
  PRIVATE
    stl_helper
)
set_property(TARGET operator_registry PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(monkey_brain_core::operator_registry ALIAS operator_registry)

add_library(expression_builder
  src/expression_builder/src/build_expression.cpp
  src/expression_builder/src/build_condition.cpp
)
target_include_directories(expression_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/expression_builder/include>
  $<INSTALL_INTERFACE:include/expression_builder>
)
target_link_libraries(expression_builder
  PUBLIC
    monkey_brain_core::environment
    monkey_brain_core::expression_description
    monkey_brain_core::expression
    monkey_brain_core::operator_registry
  PRIVATE
    stl_helper
)
set_property(TARGET expression_builder PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(monkey_brain_core::expression_builder ALIAS expression_builder)

add_library(action_builder src/action_builder/src/build_actions.cpp)
target_include_directories(action_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/action_builder/include>
  $<INSTALL_INTERFACE:include/action_builder>
)
target_link_libraries(action_builder
  PUBLIC
    monkey_brain_core::environment
    monkey_brain_core::action_description
    monkey_brain_core::action
    monkey_brain_core::expression_builder
    monkey_brain_core::operator_registry
  PRIVATE
    stl_helper
)
set_property(TARGET action_builder PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(monkey_brain_core::action_builder ALIAS action_builder)

add_library(concrete_operations_factory SHARED src/operations_factory_concrete/src/concrete_operations_factory.cpp)
target_include_directories(concrete_operations_factory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/operations_factory_concrete/include>
  $<INSTALL_INTERFACE:include/operations_factory_concrete>
)
target_link_libraries(concrete_operations_factory
  PUBLIC
    monkey_brain_core::operations_factory
    monkey_brain_core::operator_registry
  PRIVATE
    monkey_brain_core::expression_builder
    monkey_brain_core::action_builder
)
add_library(monkey_brain_core::concrete_operations_factory ALIAS concrete_operations_factory)

add_library(io_plugin_registry STATIC src/io_plugin_registry/src/io_plugin_registry.cpp)
target_include_directories(io_plugin_registry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/io_plugin_registry/include>
  $<INSTALL_INTERFACE:include/io_plugin_registry>
)
target_link_libraries(io_plugin_registry
  PUBLIC
    monkey_brain_core::environment
    monkey_brain_core::io_plugin
)

add_library(test_utils INTERFACE)
target_include_directories(test_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/test_utils/include>
  $<INSTALL_INTERFACE:include/test_utils>
)
target_link_libraries(test_utils INTERFACE environment io_plugin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_cli_context test/test_cli_context.cpp)
  target_link_libraries(test_cli_context cli_context)

  ament_add_gmock(test_expression_parser test/test_expression_parser.cpp)
  target_link_libraries(test_expression_parser expression_parser test_utils)

  ament_add_gmock(test_action_parser test/test_action_parser.cpp)
  target_link_libraries(test_action_parser action_parser test_utils)

  ament_add_gmock(test_expression_builder test/test_expression_builder.cpp)
  target_link_libraries(test_expression_builder expression_builder io_plugin_registry test_utils)

  ament_add_gmock(test_actions_builder test/test_actions_builder.cpp)
  target_link_libraries(test_actions_builder action_builder io_plugin_registry test_utils)

  ament_add_gmock(test_expression_creation test/test_expression_creation.cpp)
  target_link_libraries(test_expression_creation expression_builder expression_parser io_plugin_registry test_utils)
endif()

install(
  TARGETS
    expression_description
    action_description
    expression
    action
    operations_factory
    environment
    expression_parser
    action_parser
    concrete_operations_factory
    context
    cli_context
    decision_engine
    io_plugin
    io_plugin_registry
    operator_factory
    operator_registry
    test_utils
    stl_helper
  EXPORT "export_${PROJECT_NAME}"
)

install(
  DIRECTORY src/stl_helper/include/
  DESTINATION include/stl_helper/
)

install(
  DIRECTORY src/context/include/
  DESTINATION include/context/
)

install(
  DIRECTORY src/cli_context/include/
  DESTINATION include/cli_context/
)

install(
  DIRECTORY src/decision_engine/include/
  DESTINATION include/decision_engine/
)

install(
  DIRECTORY src/io_plugin/include/
  DESTINATION include/io_plugin/
)

install(
  DIRECTORY src/expression_description/include/
  DESTINATION include/expression_description/
)

install(
  DIRECTORY src/action_description/include/
  DESTINATION include/action_description/
)

install(
  DIRECTORY src/expression/include/
  DESTINATION include/expression/
)

install(
  DIRECTORY src/action/include/
  DESTINATION include/action/
)

install(
  DIRECTORY src/operations_factory/include/
  DESTINATION include/operations_factory/
)

install(
  DIRECTORY src/operator_factory/include/
  DESTINATION include/operator_factory/
)

install(
  DIRECTORY src/operator_registry/include/
  DESTINATION include/operator_registry/
)

install(
  DIRECTORY src/environment/include/
  DESTINATION include/environment/
)

install(
  DIRECTORY src/expression_parser/include/
  DESTINATION include/expression_parser/
)

install(
  DIRECTORY src/action_parser/include/
  DESTINATION include/action_parser/
)

install(
  DIRECTORY src/operations_factory_concrete/include/
  DESTINATION include/operations_factory_concrete/
)

install(
  DIRECTORY src/io_plugin_registry/include/
  DESTINATION include/io_plugin_registry/
)

install(
  DIRECTORY src/test_utils/include/
  DESTINATION include/test_utils/
)

ament_export_targets("export_${PROJECT_NAME}")
ament_export_dependencies(veneer)

ament_package()
