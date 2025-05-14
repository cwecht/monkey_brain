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

from ament_index_python import get_package_share_path

from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Message
from rosidl_parser.definition import NamespacedType
from rosidl_parser.parser import parse_idl_file
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
from rosidl_pycommon import generate_files


def generate_monkey_brain_plugins(generator_arguments_file):
    mapping = {
        'inoutputter.cpp.em': '%s_inoutputter.cpp',
        'inout_plugin.xml.em': '%s_inout_plugin.xml',
    }
    return generate_files(
        generator_arguments_file, mapping,
        post_process_callback=prefix_with_bom_if_necessary)


def prefix_with_bom_if_necessary(content):
    try:
        content.encode('ASCII')
    except UnicodeError:
        prefix = '\ufeff' + \
            '// NOLINT: This file starts with a BOM ' + \
            'since it contain non-ASCII characters\n'
        content = prefix + content
    return content


def to_upper_case(field_name):
    return ''.join([x.upper() for x in convert_camel_case_to_lower_case_underscore(field_name)])


class FullTypeDescriptor:

    def __init__(self, type_name: str, full_path):
        self.type_name = type_name
        self.full_path = full_path


def create_full_type_decriptors(members, base=[]):
    descriptors = []
    for member in members:
        if len(members) == 1 and member.name == EMPTY_STRUCTURE_REQUIRED_MEMBER_NAME:
            continue
        full_member_name = base + [convert_camel_case_to_lower_case_underscore(member.name)]
        descriptors.append(FullTypeDescriptor(msg_type_to_str(member.type), full_member_name))
        if isinstance(member.type, NamespacedType):
            package = member.type.namespaces[0]
            idl_path = get_package_share_path(package)
            file = parse_idl_file(IdlLocator(idl_path, 'msg/' + member.type.name + '.idl'))
            all_elements = file.content.get_elements_of_type(Message)
            els = [e for e in all_elements if e.structure.namespaced_type == member.type]
            assert len(els) == 1
            descriptors += create_full_type_decriptors(els[0].structure.members, full_member_name)

    return descriptors


def create_reference_name(descriptor, prefix=''):
    ref = to_upper_case('_'.join(descriptor.full_path))
    return to_upper_case(prefix) + '_' + ref if prefix != '' else ref


def create_references(members, modifier, prefix=''):
    descriptors = create_full_type_decriptors(members)
    return ['{' + create_reference_name(d, prefix) + ', "' + d.type_name + '", ' + modifier + '}'
            for d in descriptors]


def create_member_paths(members):
    descriptors = create_full_type_decriptors(members)
    return ['{' + create_reference_name(d) + ', "' + d.type_name + '"}' for d in descriptors]


def msg_type_to_str(type_):
    if isinstance(type_, Array):
        return msg_type_to_str(type_.value_type) + '[]'
    if isinstance(type_, AbstractSequence):
        return msg_type_to_str(type_.value_type) + '[]'
    if isinstance(type_, BasicType):
        return type_.typename
    elif isinstance(type_, AbstractString):
        return 'string'
    elif isinstance(type_, AbstractWString):
        return 'wstring'
    elif isinstance(type_, NamespacedType):
        return '/'.join(type_.namespaced_name())
    else:
        assert False, type_
