# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(rosidl_runtime_cpp REQUIRED)
find_package(monkey_brain_ros_utils REQUIRED)
find_package(monkey_brain_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

macro(custom_pluginlib_export_plugin_description_file plugin_category abs_filename)
  set(filename "")
  get_filename_component(filename  "${abs_filename}" NAME)
  install(FILES ${abs_filename} DESTINATION share/${PROJECT_NAME})

  # this accumulated value is written to the ament index resource file in the
  # ament_package() call via the pluginlib hook
  set(__PLUGINLIB_CATEGORY_CONTENT__${plugin_category}
    "${__PLUGINLIB_CATEGORY_CONTENT__${plugin_category}}share/${PROJECT_NAME}/${filename}\n")
  list(APPEND __PLUGINLIB_PLUGIN_CATEGORIES ${plugin_category})  # duplicates are removes on use
endmacro()


set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_monkey_brain/${PROJECT_NAME}")
set(_generated_headers "")
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _msg_type_name)

  list(APPEND _generated_headers
    "${_output_path}/${_parent_folder}/${_msg_type_name}_inoutputter.cpp"
  )
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_monkey_brain_BIN}"
  ${rosidl_generator_monkey_brain_GENERATOR_FILES}
  "${rosidl_generator_monkey_brain_TEMPLATE_DIR}/inoutputter.cpp.em"
  "${rosidl_generator_monkey_brain_TEMPLATE_DIR}/inout_plugin.xml.em"
  "${rosidl_generator_monkey_brain_TEMPLATE_DIR}/subscriber.cpp.em"
  "${rosidl_generator_monkey_brain_TEMPLATE_DIR}/publisher.cpp.em"
  "${rosidl_generator_monkey_brain_TEMPLATE_DIR}/service_client.cpp.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_monkey_brain__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_monkey_brain_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
  TYPE_DESCRIPTION_TUPLES "${${rosidl_generate_interfaces_TARGET}__DESCRIPTION_TUPLES}"
)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${_generated_headers}
  COMMAND Python3::Interpreter
  ARGS ${rosidl_generator_monkey_brain_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${target_dependencies} "${rosidl_generate_interfaces_TARGET}__rosidl_generator_type_description"
  COMMENT "Generating C++ code for ROS interfaces"
  VERBATIM
)

foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _msg_type_name)

  add_library(${_msg_type_name}_inoutputter SHARED "${_output_path}/${_parent_folder}/${_msg_type_name}_inoutputter.cpp")
  target_include_directories(${_msg_type_name}_inoutputter PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")
  target_link_libraries(${_msg_type_name}_inoutputter PRIVATE ${rosidl_generate_interfaces_TARGET}${_target_suffix})

  target_link_libraries(${_msg_type_name}_inoutputter
        PRIVATE
        rclcpp::rclcpp
        monkey_brain_core::io_plugin
        monkey_brain_ros_utils::monkey_brain_ros_utils
        pluginlib::pluginlib
  )
  if("${_abs_idl_file}" MATCHES ".*/action/.*")
    find_package(rclcpp_action REQUIRED)
    target_link_libraries(${_msg_type_name}_inoutputter PRIVATE rclcpp_action::rclcpp_action)
  endif()

  set_target_properties(${_msg_type_name}_inoutputter PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN 1
  )

  add_dependencies(
    ${_msg_type_name}_inoutputter
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_cpp
  )
endforeach()

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
    get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
    get_filename_component(_parent_folder "${_parent_folder}" NAME)
    get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
    string_camel_case_to_lower_case_underscore("${_idl_name}" _msg_type_name)

    ament_export_targets(export_${_msg_type_name}_inoutputter)
    install(
      TARGETS ${_msg_type_name}_inoutputter
      EXPORT export_${_msg_type_name}_inoutputter
    )

    custom_pluginlib_export_plugin_description_file(monkey_brain ${_output_path}/${_parent_folder}/${_msg_type_name}_inout_plugin.xml)

    ament_export_libraries(${_msg_type_name}_inoutputter)
  endforeach()
endif()

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(
    TESTNAME "cppcheck_rosidl_generated_monkey_brain"
    "${_output_path}")

  find_package(ament_cmake_cpplint REQUIRED)
  get_filename_component(_cpplint_root "${_output_path}" DIRECTORY)
  ament_cpplint(
    TESTNAME "cpplint_rosidl_generated_monkey_brain"
    # the generated code might contain longer lines for templated types
    MAX_LINE_LENGTH 999
    ROOT "${_cpplint_root}"
    "${_output_path}")

  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(
    TESTNAME "uncrustify_rosidl_generated_monkey_brain"
    # the generated code might contain longer lines for templated types
    # a value of zero tells uncrustify to ignore line length
    MAX_LINE_LENGTH 0
    "${_output_path}")
endif()
