/*
Copyright 2023 Thales Alenia Space
*/

#include "systemctl_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace addons
{

SystemctlController::SystemctlController(const rclcpp::NodeOptions & options)
: Node("systemctl_node", options)
{
  RCLCPP_INFO(get_logger(), "SystemctlController constructor called");
  RCLCPP_INFO(get_logger(), "Node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "Node namespace: %s", get_namespace());
  
  // Debug: List all parameters to see what's actually loaded
  auto param_names = this->list_parameters({}, 0);
  RCLCPP_INFO(get_logger(), "Total parameters loaded: %zu", param_names.names.size());
  for (const auto& name : param_names.names) {
    RCLCPP_INFO(get_logger(), "Parameter found: %s", name.c_str());
  }
  
  // Changed from simple list to structured configuration
  // Each service can now have its own namespace and topic name
  start_service_name_ = declare_parameter<std::string>("services_name.start", "start_service");
  stop_service_name_ = declare_parameter<std::string>("services_name.stop", "stop_service");
  restart_service_name_ =
    declare_parameter<std::string>("services_name.restart", "restart_service");
  query_service_name_ = declare_parameter<std::string>("services_name.query", "query_service");
  
  // Get service configurations - each service can have its own fully qualified path
  auto service_configs = declare_parameter<std::vector<std::string>>("sys_services", std::vector<std::string>());
  
  RCLCPP_INFO(get_logger(), "Found %zu service configurations", service_configs.size());
  
  // Parse service configurations
  for (const auto& config : service_configs) {
    // Expected format: "systemd_service_name:fully_qualified_path" or just "systemd_service_name"
    std::string systemd_service;
    std::string service_path = "";
    
    size_t colon_pos = config.find(':');
    if (colon_pos != std::string::npos) {
      systemd_service = config.substr(0, colon_pos);
      service_path = config.substr(colon_pos + 1);
      // Ensure path ends with '/' for consistent service naming
      if (!service_path.empty() && service_path.back() != '/') {
        service_path += "/";
      }
    } else {
      systemd_service = config;
      service_path = systemd_service + "/"; // Default behavior
    }
    
    sys_services_list.push_back(systemd_service);
    service_namespaces_[systemd_service] = service_path;
    
    RCLCPP_INFO(get_logger(), "Configured service '%s' with path '%s'", 
                systemd_service.c_str(), service_path.c_str());
  }

  auto systemd_method_cb = [this](std::string systemd_unit, std::string systemd_method) {
      return [this, systemd_unit,
               systemd_method](const std::shared_ptr<std_srvs::srv::Trigger::Request> in,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
               this->service_call(in, out, systemd_method, systemd_unit);
             };
    };
  auto systemd_query_cb = [this](std::string systemd_unit, std::string systemd_method) {
      return [this, systemd_unit,
               systemd_method](const std::shared_ptr<std_srvs::srv::Trigger::Request> in,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
               this->query_status(in, out, systemd_method, systemd_unit);
             };
    };

  for (const std::string& sys_ser : sys_services_list) {
    std::string service_prefix = service_namespaces_[sys_ser];
    
    RCLCPP_INFO(get_logger(), "Creating services for '%s' with prefix '%s'", 
                sys_ser.c_str(), service_prefix.c_str());
    
    start_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        service_prefix + start_service_name_, 
        systemd_method_cb(sys_ser, "StartUnit")));
    stop_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        service_prefix + stop_service_name_,
        systemd_method_cb(sys_ser, "StopUnit")));
    restart_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        service_prefix + restart_service_name_, 
        systemd_method_cb(sys_ser, "RestartUnit")));
    query_srvs_.push_back(
      create_service<std_srvs::srv::Trigger>(
        service_prefix + query_service_name_, 
        systemd_query_cb(sys_ser, "ActiveState")));
        
    RCLCPP_INFO(get_logger(), "Created services: %s, %s, %s, %s", 
                (service_prefix + start_service_name_).c_str(),
                (service_prefix + stop_service_name_).c_str(),
                (service_prefix + restart_service_name_).c_str(),
                (service_prefix + query_service_name_).c_str());
  }
}

void SystemctlController::service_call(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response,
  const std::string & method,
  std::string sys_ser)
{
  sd_bus * bus = NULL;
  sd_bus_error err = SD_BUS_ERROR_NULL;
  sd_bus_message * msg = nullptr;
  int ret = sd_bus_open_user(&bus);

  // analyze the sd_bus_default_system
  if (ret < 0) {
    RCLCPP_WARN(get_logger(), "Could not open the User_Bus. Return: %d ", ret);
    return;
  }
  std::string appendix(".service");
  sys_ser.append(appendix);
  ret = sd_bus_call_method(
    bus,                                 /* <bus>       */
    "org.freedesktop.systemd1",          /* <service>   */
    "/org/freedesktop/systemd1",         /* <path>      */
    "org.freedesktop.systemd1.Manager",  /* <interface> */
    method.c_str(),                      /* <method>    */
    &err,                                /* object to return error in */
    &msg,                                /* return message on success */
    "ss",                                /* <input_signature (string-string)> */
    sys_ser.c_str(), "replace");         /* <arguments...> */

  if (ret < 0) {
    RCLCPP_WARN(get_logger(), "%s call failed. Return: %d. ", method.c_str(), ret);
    std::string error_str(err.message);
    std::string debug_str(method + " call failed: " + error_str);
    response->success = false;
    response->message = debug_str;
    return;
  }
  RCLCPP_INFO(get_logger(), "Systemd method: %s called successfully.", method.c_str());
  // Clean up the memory:
  sd_bus_error_free(&err);
  sd_bus_message_unref(msg);
  sd_bus_unref(bus);
  std::string out_str("Systemd method: " + method + " called successfully.");
  response->success = true;
  response->message = out_str;
}

void SystemctlController::query_status(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response,
  const std::string & property,
  std::string sys_res)
{
  sd_bus * bus = NULL;
  sd_bus_error err = SD_BUS_ERROR_NULL;
  char * state = nullptr;
  int ret = sd_bus_open_user(&bus);

  if (ret < 0) {
    RCLCPP_WARN(get_logger(), "Could not open the Bus. Return: %d ", ret);
    return;
  }
  std::string path_str("/org/freedesktop/systemd1/unit/" + sys_res + "_2eservice");
  ret = sd_bus_get_property_string(
    bus,
    "org.freedesktop.systemd1",
    path_str.c_str(),
    "org.freedesktop.systemd1.Unit",
    property.c_str(),
    &err,
    &state);

  if (ret < 0) {
    std::string error_str(err.message);
    RCLCPP_WARN(
      get_logger(), "Could NOT query the status of the service. Error: %s. Return: %d.",
      error_str.c_str(), ret);
    response->success = false;
    return;
  }

  std::string state_str(state);
  std::string out_str("Systemd service: " + sys_res + " status: " + state_str);
  RCLCPP_INFO(get_logger(), "Service %s Status: %s.", sys_res.c_str(), state_str.c_str());
  // Clean up
  sd_bus_error_free(&err);
  sd_bus_unref(bus);
  free(state);
  response->success = true;
  response->message = out_str;
}
}  // namespace addons

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  
  // Check for namespace argument
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--ros-args" && i + 2 < argc) {
      std::string next_arg(argv[i + 1]);
      if (next_arg == "-r") {
        std::string remap_arg(argv[i + 2]);
        if (remap_arg.find("__ns:=") == 0) {
          // Extract namespace from __ns:=/namespace format
          std::string ns = remap_arg.substr(6);
          if (!ns.empty() && ns[0] != '/') {
            ns = "/" + ns;
          }
          // The namespace will be handled by rclcpp automatically through remapping
        }
      }
    }
  }
  
  auto node = std::make_shared<addons::SystemctlController>(options);
  
  RCLCPP_INFO(node->get_logger(), "SystemCtl Controller Node started");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
