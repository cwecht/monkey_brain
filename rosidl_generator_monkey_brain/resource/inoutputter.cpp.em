@
@#######################################################################
@# EmPy template for generating _<idl>.py files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@{
import_statements = set()
}@
@
@#######################################################################
@# Handle services
@#######################################################################
@{
from rosidl_parser.definition import Service
}@
@[for service in content.get_elements_of_type(Service)]@
@{
TEMPLATE(
    'service_client.cpp.em',
    package_name=package_name, interface_path=interface_path, service=service,
    import_statements=import_statements)
}@
@[end for]@
@
@
@#######################################################################
@# Handle messages
@#######################################################################
@{
from rosidl_parser.definition import Message
}@
@[for message in content.get_elements_of_type(Message)]@
@{
TEMPLATE(
    'subscriber.cpp.em',
    package_name=package_name, interface_path=interface_path, message=message,
    import_statements=import_statements)
TEMPLATE(
    'publisher.cpp.em',
    package_name=package_name, interface_path=interface_path, message=message,
    import_statements=import_statements)
}@
@[end for]@
@
@#######################################################################
@# Handle services
@#######################################################################
@{
from rosidl_parser.definition import Action
}@
@[for action in content.get_elements_of_type(Action)]@
@{
TEMPLATE(
    'action_client.cpp.em',
    package_name=package_name, interface_path=interface_path, action=action,
    import_statements=import_statements)
}@
@[end for]@
@
