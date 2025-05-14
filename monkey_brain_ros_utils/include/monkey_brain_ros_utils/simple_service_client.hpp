#ifndef MONKEY_BRAIN_ROS_UTILS_SIMPLE_SERVICE_CLIENT_HPP
#define MONKEY_BRAIN_ROS_UTILS_SIMPLE_SERVICE_CLIENT_HPP

#include <rclcpp/client.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/qos.hpp>

namespace monkey_brain_ros_utils
{

template<typename ServiceT>
class SimpleClient : public rclcpp::ClientBase
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  using CallbackType = std::function<void (const Response &)>;

  RCLCPP_SMART_PTR_DEFINITIONS(SimpleClient)

  /// Default constructor.
  /**
   * The constructor for a Client is almost never called directly.
   * Instead, clients should be instantiated through the function
   * rclcpp::create_client().
   *
   * \param[in] node_base NodeBaseInterface pointer that is used in part of the setup.
   * \param[in] node_graph The node graph interface of the corresponding node.
   * \param[in] service_name Name of the topic to publish to.
   * \param[in] client_options options for the subscription.
   */
  SimpleClient(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string & service_name, CallbackType callback,
    rcl_client_options_t & client_options)
  : ClientBase(node_base, node_graph), mCallback{std::move(callback)}
  {
    using rosidl_typesupport_cpp::get_service_type_support_handle;
    auto service_type_support_handle =
      get_service_type_support_handle<ServiceT>();
    rcl_ret_t ret = rcl_client_init(
      this->get_client_handle().get(),
      this->get_rcl_node_handle(),
      service_type_support_handle,
      service_name.c_str(),
      &client_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = this->get_rcl_node_handle();
        // this will throw on any validation problem
        rcl_reset_error();
        rclcpp::expand_topic_or_service_name(
          service_name,
          rcl_node_get_name(rcl_node_handle),
          rcl_node_get_namespace(rcl_node_handle),
          true);
      }
      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
    }
  }

  /// Take the next response for this client.
  /**
   * \sa ClientBase::take_type_erased_response().
   *
   * \param[out] response_out The reference to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associate the response
   *   to a specific request.
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  bool
  take_response(typename ServiceT::Response & response_out, rmw_request_id_t & request_header_out)
  {
    return this->take_type_erased_response(&response_out, request_header_out);
  }

  /// Create a shared pointer with the response type
  /**
   * \return shared pointer with the response type
   */
  std::shared_ptr<void>
  create_response() override
  {
    return response_;
  }

  /// Create a shared pointer with a rmw_request_id_t
  /**
   * \return shared pointer with a rmw_request_id_t
   */
  std::shared_ptr<rmw_request_id_t>
  create_request_header() override
  {
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)
    return std::shared_ptr<rmw_request_id_t>(&request_id_, [](rmw_request_id_t *) {});
  }

  /// Handle a server response
  /**
    * \param[in] request_header used to check if the sequence number is valid
    * \param[in] response message with the server response
   */
  void
  handle_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response) override
  {
    if (pending_request_ != request_header->sequence_number) {
      return;
    }

    auto typed_response = std::static_pointer_cast<typename ServiceT::Response>(
      std::move(response));

    mCallback(*typed_response);
    pending_request_.reset();
  }

  /// Send a request to the service server.
  /**
   *
   * \param[in] request request to be send.
   */
  void async_send_request(Request & request)
  {
    int64_t sequence_number = 0;
    const rcl_ret_t ret = rcl_send_request(get_client_handle().get(), &request, &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }
    pending_request_.emplace(sequence_number);
  }

protected:
  RCLCPP_DISABLE_COPY(SimpleClient)
  CallbackType mCallback;
  std::optional<uint64_t> pending_request_;
  rmw_request_id_t request_id_{};
  std::shared_ptr<void> response_{new typename ServiceT::Response()};
};

template<typename ServiceT, typename Callback>
typename SimpleClient<ServiceT>::SharedPtr
create_simple_client(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph,
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  Callback && callback,
  const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos.get_rmw_qos_profile();

  auto cli = SimpleClient<ServiceT>::make_shared(
    node_base.get(),
    node_graph,
    service_name,
    std::forward<Callback>(callback),
    options);

  auto cli_base_ptr = std::dynamic_pointer_cast<rclcpp::ClientBase>(cli);
  node_services->add_client(cli_base_ptr, group);
  return cli;
}

template<typename ServiceT, typename Callback>
typename SimpleClient<ServiceT>::SharedPtr
create_simple_client(
  rclcpp::Node * node,
  const std::string & service_name,
  Callback && callback,
  const rclcpp::QoS & qos = rclcpp::ServicesQoS{},
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return create_simple_client<ServiceT, Callback>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface(),
    rclcpp::extend_name_with_sub_namespace(service_name, node->get_sub_namespace()),
    std::forward<Callback>(callback),
    qos,
    group);
}

} // namespace monkey_brain_ros_utils
#endif // MONKEY_BRAIN_ROS_UTILS_SIMPLE_SERVICE_CLIENT_HPP
