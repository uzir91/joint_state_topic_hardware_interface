// Copyright 2026
// Licensed under the Apache License, Version 2.0

#include "cm_topic_hardware_component/humble_system_interface_compat.hpp"

#include <cctype>
#include <limits>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace cm_topic_hardware_component
{

HumbleStateInterfaceHandle::HumbleStateInterfaceHandle(
  std::string name,
  const HandleDataType data_type)
: name_(std::move(name)),
  data_type_(data_type)
{
}

const std::string &
HumbleStateInterfaceHandle::get_name() const noexcept
{
  return name_;
}

HandleDataType
HumbleStateInterfaceHandle::get_data_type() const noexcept
{
  return data_type_;
}

HumbleSystemInterfaceCompat::~HumbleSystemInterfaceCompat()
{
  stop_executor();
}

HumbleSystemInterfaceCompat::CallbackReturn
HumbleSystemInterfaceCompat::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    state_values_.clear();
    command_values_.clear();
    state_interfaces_.clear();
    command_interfaces_.clear();
    state_handles_.clear();

    append_component_interfaces(info.joints);
    append_component_interfaces(info.sensors);
    append_component_interfaces(info.gpios);

    std::string node_namespace;
    const auto namespace_it =
      info.hardware_parameters.find("node_namespace");

    if (namespace_it != info.hardware_parameters.end())
    {
      node_namespace = namespace_it->second;
    }

    rclcpp::NodeOptions node_options;
    node_options.use_global_arguments(false);

    // Keep the hardware component name so "~/names" and "~/values" resolve
    // exactly like the newer ros2_control-owned hardware node.
    node_ = std::make_shared<rclcpp::Node>(
      sanitize_node_name(info.name),
      node_namespace,
      node_options);

    start_executor();
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "cm_topic_humble_system_interface_compat"),
      "Failed to initialize Humble compatibility wrapper '%s': %s",
      info.name.c_str(),
      error.what());

    stop_executor();
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HumbleSystemInterfaceCompat::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(state_interfaces_.size());

  for (const auto & interface : state_interfaces_)
  {
    auto value_it = state_values_.find(interface.full_name);

    if (value_it == state_values_.end())
    {
      throw std::runtime_error(
              "Missing state storage for '" +
              interface.full_name + "'");
    }

    interfaces.emplace_back(
      interface.component_name,
      interface.interface_name,
      &value_it->second);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
HumbleSystemInterfaceCompat::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(command_interfaces_.size());

  for (const auto & interface : command_interfaces_)
  {
    auto value_it = command_values_.find(interface.full_name);

    if (value_it == command_values_.end())
    {
      throw std::runtime_error(
              "Missing command storage for '" +
              interface.full_name + "'");
    }

    interfaces.emplace_back(
      interface.component_name,
      interface.interface_name,
      &value_it->second);
  }

  return interfaces;
}

rclcpp::Node::SharedPtr
HumbleSystemInterfaceCompat::get_node() const noexcept
{
  return node_;
}

bool HumbleSystemInterfaceCompat::has_state(
  const std::string & interface_name) const noexcept
{
  return state_values_.find(interface_name) !=
         state_values_.end();
}

HumbleSystemInterfaceCompat::StateInterfaceHandle
HumbleSystemInterfaceCompat::get_state_interface_handle(
  const std::string & interface_name) const
{
  return state_handles_.at(interface_name);
}

void HumbleSystemInterfaceCompat::set_state(
  const StateInterfaceHandle & handle,
  const double value,
  const bool /*force_cast*/)
{
  if (!handle)
  {
    throw std::invalid_argument(
            "Cannot set state through a null handle");
  }

  state_values_.at(handle->get_name()) = value;
}

void HumbleSystemInterfaceCompat::set_state(
  const StateInterfaceHandle & handle,
  const bool value,
  const bool /*force_cast*/)
{
  if (!handle)
  {
    throw std::invalid_argument(
            "Cannot set state through a null handle");
  }

  state_values_.at(handle->get_name()) =
    value ? 1.0 : 0.0;
}

void HumbleSystemInterfaceCompat::shutdown_compat_node() noexcept
{
  stop_executor();
}

std::string HumbleSystemInterfaceCompat::sanitize_node_name(
  const std::string & value)
{
  std::string result;
  result.reserve(value.size() + 1);

  for (const char character : value)
  {
    const auto unsigned_character =
      static_cast<unsigned char>(character);

    if (std::isalnum(unsigned_character) != 0 ||
      character == '_')
    {
      result.push_back(character);
    }
    else
    {
      result.push_back('_');
    }
  }

  if (result.empty())
  {
    result = "cm_topic_hardware";
  }
  else if (
    std::isdigit(
      static_cast<unsigned char>(result.front())) != 0)
  {
    result.insert(result.begin(), '_');
  }

  return result;
}

HandleDataType HumbleSystemInterfaceCompat::parse_data_type(
  const std::string & data_type)
{
  if (data_type.empty() || data_type == "double")
  {
    return HandleDataType::DOUBLE;
  }

  if (data_type == "bool")
  {
    return HandleDataType::BOOL;
  }

  return HandleDataType::UNSUPPORTED;
}

double HumbleSystemInterfaceCompat::initial_interface_value(
  const hardware_interface::InterfaceInfo & interface)
{
  if (interface.initial_value.empty())
  {
    if (parse_data_type(interface.data_type) ==
      HandleDataType::BOOL)
    {
      return 0.0;
    }

    return std::numeric_limits<double>::quiet_NaN();
  }

  return std::stod(interface.initial_value);
}

void HumbleSystemInterfaceCompat::append_component_interfaces(
  const std::vector<hardware_interface::ComponentInfo> &
  components)
{
  for (const auto & component : components)
  {
    for (const auto & interface : component.state_interfaces)
    {
      const std::string full_name =
        component.name + "/" + interface.name;

      const auto inserted = state_values_.emplace(
        full_name,
        initial_interface_value(interface));

      if (!inserted.second)
      {
        throw std::runtime_error(
                "Duplicate state interface '" +
                full_name + "'");
      }

      state_interfaces_.push_back(
        InterfaceStorage{
          component.name,
          interface.name,
          full_name,
          interface.data_type,
          inserted.first->second});

      const auto handle_inserted =
        state_handles_.emplace(
        full_name,
        std::make_shared<HumbleStateInterfaceHandle>(
          full_name,
          parse_data_type(interface.data_type)));

      if (!handle_inserted.second)
      {
        throw std::runtime_error(
                "Duplicate state metadata handle '" +
                full_name + "'");
      }
    }

    for (const auto & interface : component.command_interfaces)
    {
      const std::string full_name =
        component.name + "/" + interface.name;

      const auto inserted = command_values_.emplace(
        full_name,
        initial_interface_value(interface));

      if (!inserted.second)
      {
        throw std::runtime_error(
                "Duplicate command interface '" +
                full_name + "'");
      }

      command_interfaces_.push_back(
        InterfaceStorage{
          component.name,
          interface.name,
          full_name,
          interface.data_type,
          inserted.first->second});
    }
  }
}

void HumbleSystemInterfaceCompat::start_executor()
{
  if (!node_)
  {
    throw std::runtime_error(
            "Cannot start executor without an internal node");
  }

  executor_ =
    std::make_unique<
    rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_node(node_);

  executor_thread_ = std::thread(
    [this]()
    {
      try
      {
        executor_->spin();
      }
      catch (const std::exception & error)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "cm_topic_humble_system_interface_compat"),
          "Internal hardware executor stopped: %s",
          error.what());
      }
    });
}

void HumbleSystemInterfaceCompat::stop_executor() noexcept
{
  if (executor_)
  {
    executor_->cancel();
  }

  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  if (executor_ && node_)
  {
    try
    {
      executor_->remove_node(node_);
    }
    catch (...)
    {
      // Destruction must not throw.
    }
  }

  executor_.reset();
  node_.reset();
}

}  // namespace cm_topic_hardware_component
