// Copyright 2026
// Licensed under the Apache License, Version 2.0

#include <joint_state_topic_hardware_interface/humble_system_interface_compat.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace joint_state_topic_hardware_interface
{

HumbleSystemInterfaceCompat::~HumbleSystemInterfaceCompat()
{
  stop_executor();
}

HumbleSystemInterfaceCompat::CallbackReturn HumbleSystemInterfaceCompat::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    build_compat_hardware_info(info);
    initialize_interface_storage(info);

    std::string node_namespace;
    const auto namespace_it = info.hardware_parameters.find("node_namespace");
    if (namespace_it != info.hardware_parameters.end())
    {
      node_namespace = namespace_it->second;
    }

    rclcpp::NodeOptions node_options;
    node_options.use_global_arguments(false);

    const auto node_name = sanitize_node_name(info.name + "_hardware_compat");
    node_ = std::make_shared<rclcpp::Node>(node_name, node_namespace, node_options);

    start_executor();
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("humble_system_interface_compat"),
      "Failed to initialize hardware compatibility wrapper '%s': %s",
      info.name.c_str(), error.what());
    stop_executor();
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HumbleSystemInterfaceCompat::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  std::size_t interface_count = 0;
  for (const auto & joint : info_.joints)
  {
    interface_count += joint.state_interfaces.size();
  }
  interfaces.reserve(interface_count);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      const auto full_name = make_interface_name(joint.name, interface.name);
      auto value_it = state_values_.find(full_name);
      if (value_it == state_values_.end())
      {
        throw std::runtime_error("Missing state interface storage for '" + full_name + "'");
      }

      interfaces.emplace_back(joint.name, interface.name, &value_it->second);
    }
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
HumbleSystemInterfaceCompat::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  std::size_t interface_count = 0;
  for (const auto & joint : info_.joints)
  {
    interface_count += joint.command_interfaces.size();
  }
  interfaces.reserve(interface_count);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.command_interfaces)
    {
      const auto full_name = make_interface_name(joint.name, interface.name);
      auto value_it = command_values_.find(full_name);
      if (value_it == command_values_.end())
      {
        throw std::runtime_error("Missing command interface storage for '" + full_name + "'");
      }

      interfaces.emplace_back(joint.name, interface.name, &value_it->second);
    }
  }

  return interfaces;
}

const HumbleSystemInterfaceCompat::HardwareInfoCompat &
HumbleSystemInterfaceCompat::get_hardware_info() const noexcept
{
  return compat_info_;
}

rclcpp::Node::SharedPtr HumbleSystemInterfaceCompat::get_node() const noexcept
{
  return node_;
}

bool HumbleSystemInterfaceCompat::has_state(const std::string & interface_name) const noexcept
{
  return state_values_.find(interface_name) != state_values_.end();
}

double HumbleSystemInterfaceCompat::get_state(const std::string & interface_name) const
{
  return state_values_.at(interface_name);
}

void HumbleSystemInterfaceCompat::set_state(
  const std::string & interface_name, const double value)
{
  state_values_.at(interface_name) = value;
}

bool HumbleSystemInterfaceCompat::has_command(const std::string & interface_name) const noexcept
{
  return command_values_.find(interface_name) != command_values_.end();
}

double HumbleSystemInterfaceCompat::get_command(const std::string & interface_name) const
{
  return command_values_.at(interface_name);
}

void HumbleSystemInterfaceCompat::set_command(
  const std::string & interface_name, const double value)
{
  command_values_.at(interface_name) = value;
}

void HumbleSystemInterfaceCompat::shutdown_compat_node() noexcept
{
  stop_executor();
}

std::string HumbleSystemInterfaceCompat::make_interface_name(
  const std::string & component_name, const std::string & interface_name)
{
  return component_name + "/" + interface_name;
}

std::string HumbleSystemInterfaceCompat::sanitize_node_name(const std::string & value)
{
  std::string result;
  result.reserve(value.size() + 1);

  for (const auto character : value)
  {
    const auto unsigned_character = static_cast<unsigned char>(character);
    if (std::isalnum(unsigned_character) != 0 || character == '_')
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
    result = "hardware_compat";
  }
  else if (std::isdigit(static_cast<unsigned char>(result.front())) != 0)
  {
    result.insert(result.begin(), '_');
  }

  return result;
}

double HumbleSystemInterfaceCompat::initial_interface_value(
  const hardware_interface::InterfaceInfo & interface)
{
  if (interface.initial_value.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return std::stod(interface.initial_value);
}

void HumbleSystemInterfaceCompat::build_compat_hardware_info(
  const hardware_interface::HardwareInfo & info)
{
  compat_info_.name = info.name;
  compat_info_.type = info.type;
  compat_info_.hardware_plugin_name = info.hardware_class_type;
  compat_info_.hardware_parameters = info.hardware_parameters;
  compat_info_.joints = info.joints;
  compat_info_.mimic_joints.clear();

  for (std::size_t joint_index = 0; joint_index < info.joints.size(); ++joint_index)
  {
    const auto & joint = info.joints[joint_index];
    const auto mimic_it = joint.parameters.find("mimic");
    if (mimic_it == joint.parameters.end())
    {
      continue;
    }

    const auto mimicked_joint_it = std::find_if(
      info.joints.begin(), info.joints.end(),
      [&mimic_it](const hardware_interface::ComponentInfo & candidate) {
        return candidate.name == mimic_it->second;
      });

    if (mimicked_joint_it == info.joints.end())
    {
      throw std::runtime_error(
        "Mimicked joint '" + mimic_it->second + "' for joint '" + joint.name + "' was not found");
    }

    MimicJoint mimic_joint;
    mimic_joint.joint_index = joint_index;
    mimic_joint.mimicked_joint_index = static_cast<std::size_t>(
      std::distance(info.joints.begin(), mimicked_joint_it));

    const auto multiplier_it = joint.parameters.find("multiplier");
    if (multiplier_it != joint.parameters.end())
    {
      mimic_joint.multiplier = std::stod(multiplier_it->second);
    }

    const auto offset_it = joint.parameters.find("offset");
    if (offset_it != joint.parameters.end())
    {
      mimic_joint.offset = std::stod(offset_it->second);
    }

    compat_info_.mimic_joints.push_back(mimic_joint);
  }
}

void HumbleSystemInterfaceCompat::initialize_interface_storage(
  const hardware_interface::HardwareInfo & info)
{
  state_values_.clear();
  command_values_.clear();

  for (const auto & joint : info.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      const auto full_name = make_interface_name(joint.name, interface.name);
      const auto inserted = state_values_.emplace(full_name, initial_interface_value(interface));
      if (!inserted.second)
      {
        throw std::runtime_error("Duplicate state interface '" + full_name + "'");
      }
    }

    for (const auto & interface : joint.command_interfaces)
    {
      const auto full_name = make_interface_name(joint.name, interface.name);
      const auto inserted = command_values_.emplace(
        full_name, initial_interface_value(interface));

      if (!inserted.second)
      {
        throw std::runtime_error("Duplicate command interface '" + full_name + "'");
      }
    }
  }
}

void HumbleSystemInterfaceCompat::start_executor()
{
  if (!node_)
  {
    throw std::runtime_error("Cannot start executor without an internal node");
  }

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::thread([this]() {
    try
    {
      executor_->spin();
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("humble_system_interface_compat"),
        "Internal hardware executor stopped with an exception: %s", error.what());
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
      // Destructors must not throw. The executor is already stopped.
    }
  }

  executor_.reset();
  node_.reset();
}

}  // namespace joint_state_topic_hardware_interface