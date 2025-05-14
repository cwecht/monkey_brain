cmake_minimum_required(VERSION 3.8)
project(monkey_brain_scxml)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wconversion)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(monkey_brain_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(veneer REQUIRED)

add_library(state_machine_description INTERFACE)
target_include_directories(state_machine_description INTERFACE src/state_machine_description/include)
target_link_libraries(state_machine_description
  INTERFACE
    monkey_brain_core::action_description
    monkey_brain_core::expression_description
)

add_library(test_utils INTERFACE)
target_include_directories(test_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/test_utils/include>
  $<INSTALL_INTERFACE:include>
)

add_library(scxml_parser src/parse_scxml/src/parse_scxml.cpp)
target_include_directories(scxml_parser PUBLIC src/parse_scxml/include)
target_link_libraries(scxml_parser
  PUBLIC
    tinyxml2::tinyxml2
    state_machine_description
    monkey_brain_core::expression_parser
    monkey_brain_core::action_parser
)
set_property(TARGET scxml_parser PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(states INTERFACE)
target_include_directories(states INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/states/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(states INTERFACE monkey_brain_core::expression monkey_brain_core::action)

add_library(state_machine_observer INTERFACE)
target_include_directories(state_machine_observer INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/state_machine_observer/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(state_machine_observer INTERFACE states monkey_brain_core::context)

add_library(state_builder src/state_builder/src/build_states.cpp)
target_include_directories(state_builder PUBLIC src/state_builder/include PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(state_builder PUBLIC states state_machine_description monkey_brain_core::operations_factory)
set_property(TARGET state_builder PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(state_machine src/state_machine/src/state_machine.cpp)
target_include_directories(state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/state_machine/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(state_machine PUBLIC
        monkey_brain_core::decision_engine
        monkey_brain_core::environment
        states state_machine_observer
        veneer::veneer
)
set_property(TARGET state_machine PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(state_machine_operators STATIC src/state_machine_operators/src/in_state_operator.cpp)
target_include_directories(state_machine_operators PUBLIC src/state_machine_operators/include)
target_link_libraries(state_machine_operators
  PRIVATE
    state_machine monkey_brain_core::expression
    monkey_brain_core::stl_helper
)
set_property(TARGET state_machine_operators PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(validate_state_machine STATIC
  src/validate_state_machine/src/validate_state_machine.cpp
)
target_include_directories(validate_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/validate_state_machine/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(validate_state_machine
  PUBLIC
    monkey_brain_core::environment
    state_machine_description
)
set_property(TARGET validate_state_machine PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(scxml_state_machine_factory SHARED src/scxml_state_machine_factory/src/scxml_state_machine_factory.cpp)
target_include_directories(scxml_state_machine_factory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/scxml_state_machine_factory/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(scxml_state_machine_factory
  PRIVATE
    state_machine
    state_builder
    scxml_parser
    state_machine_operators
    validate_state_machine
    pluginlib::pluginlib
    monkey_brain_core::context
    monkey_brain_core::operator_registry
    veneer::veneer
)

pluginlib_export_plugin_description_file(monkey_brain sm_plugin.xml)

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

  ament_add_gmock(test_statemachine test/test_statemachine.cpp)
  target_link_libraries(test_statemachine state_machine monkey_brain_core::test_utils test_utils)

  ament_add_gmock(test_scxml_parser test/test_scxml_parser.cpp
          ENV TEST_DATA_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_scxml_parser scxml_parser monkey_brain_core::test_utils)

  ament_add_gmock(test_statemachine_integration test/test_statemachine_integration.cpp
          ENV TEST_DATA_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_statemachine_integration scxml_parser state_machine state_builder monkey_brain_core::test_utils)

  ament_add_gmock(test_state_builder test/test_states_builder.cpp)
  target_link_libraries(test_state_builder state_builder monkey_brain_core::test_utils)

  ament_add_gmock(test_validate_state_machine test/test_validate_state_machine.cpp)
  target_link_libraries(test_validate_state_machine validate_state_machine monkey_brain_core::test_utils)

  ament_add_gmock(test_scxml_irp test/scxml-irp-test/test_scxml_irp.cpp
          ENV SCXML_IRP_TEST_DATA_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/test/scxml-irp-test)
  target_link_libraries(test_scxml_irp
          scxml_state_machine_factory state_machine
          monkey_brain_core::concrete_operations_factory
          monkey_brain_core::io_plugin_registry
          monkey_brain_core::test_utils
  )
endif()

install(TARGETS scxml_state_machine_factory state_machine_observer states test_utils
  EXPORT "export_${PROJECT_NAME}"
  INCLUDES DESTINATION include
)

install(
  DIRECTORY src/states/include/
  DESTINATION include/
)

install(
  DIRECTORY src/state_machine_observer/include/
  DESTINATION include/
)

install(
  DIRECTORY src/test_utils/include/
  DESTINATION include/
)

ament_export_targets("export_${PROJECT_NAME}")

ament_package()
