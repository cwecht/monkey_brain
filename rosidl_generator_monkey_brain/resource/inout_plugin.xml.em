@{
from rosidl_parser.definition import Message
from rosidl_parser.definition import Service
from rosidl_parser.definition import Action 
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
library_name = convert_camel_case_to_lower_case_underscore(interface_path.stem)
}@

<library path="@(library_name)_inoutputter">
@[if len(content.get_elements_of_type(Service)) == 0 and len(content.get_elements_of_type(Action)) == 0]
@[for message in content.get_elements_of_type(Message)]
  <class name="@(package_name)/@(message.structure.namespaced_type.name)/Subscriber"
	 type="monkey_brain_core::IOPluginFactoryImpl<@(message.structure.namespaced_type.name)SubscriberPlugin>"
	 base_class_type="monkey_brain_core::IOPluginFactory">
    <description>Creates a subscriber for a topic of type @(package_name)/msg/@(message.structure.namespaced_type.name).

parameters: 0</description>
  </class>
 <class name="@(package_name)/@(message.structure.namespaced_type.name)/Publisher"
	 type="monkey_brain_core::IOPluginFactoryImpl<@(message.structure.namespaced_type.name)PublisherPlugin>"
	 base_class_type="monkey_brain_core::IOPluginFactory">
    <description>Creates a publisher for a topic of type @(package_name)/msg/@(message.structure.namespaced_type.name).

parameters: 0</description>
  </class>
@[end for]@
@[else]@
@[for service in content.get_elements_of_type(Service)]
  <class name="@(package_name)/@(service.namespaced_type.name)/ServiceClient"
	 type="monkey_brain_core::IOPluginFactoryImpl<@(service.namespaced_type.name)ServiceClient>"
	 base_class_type="monkey_brain_core::IOPluginFactory">
    <description>Creates a service client of type @(package_name)/@(service.namespaced_type.name).
It provides a variable &lt;prefix&gt;/is_ready which indicates whether the service is online (a matching service server is online).
As long a the service is not online, this variable is periodically updated by a timer with a period of &lt;retry_period&gt;
ms. As soon as the service is available the a &lt;prefix&gt;/is_ready event is triggered.

parameters: 1
 - retry_period: the retry period of connecting to the service server.</description>
  </class>
@[end for]@
@[for action in content.get_elements_of_type(Action)]
  <class name="@(package_name)/@(action.namespaced_type.name)/ActionClient"
	 type="monkey_brain_core::IOPluginFactoryImpl<@(action.namespaced_type.name)ActionClient>"
	 base_class_type="monkey_brain_core::IOPluginFactory">
    <description>Creates an action client of type @(package_name)/@(action.namespaced_type.name).
It provides a variable &lt;prefix&gt;/is_ready which indicates whether the action is online (a matching action server is online).
As long a the action is not online, this variable is periodically updated by a timer with a period of &lt;retry_period&gt;
ms. As soon as the action is available the a &lt;prefix&gt;/is_ready event is triggered.

parameters: 1
 - retry_period: the retry period of connecting to the action server.</description>
  </class>
@[end for]@
@[end if]@
</library>
