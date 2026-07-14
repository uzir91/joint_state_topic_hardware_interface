// Copyright 2026
// Licensed under the Apache License, Version 2.0

#include "joint_state_topic_hardware_interface/humble_system_interface_compat.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace joint_state_topic_hardware_interface
{

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
    build_compat_hardware_info(info);
    initialize_interface_storage(info);

    std::string node_namespace;
    const auto namespace_it =
      info.hardware_parameters.find("node_namespace");

    if (namespace_it != info.hardware_parameters.end())
    {
      node_namespace = namespace_it->second;
    }

    rclcpp::NodeOptions node_options;
    node_options.use_global_arguments(false);

    node_ = std::make_shared<rclcpp::Node>(
      sanitize_node_name(info.name + "_hardware"),
      node_namespace,
      node_options);

    start_executor();
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "joint_state_humble_system_interface_compat"),
      "Failed to initialize compatibility wrapper '%s': %s",
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
      const auto full_name =
        make_interface_name(joint.name, interface.name);

      auto value_it = state_values_.find(full_name);
      if (value_it == state_values_.end())
      {
        throw std::runtime_error(
                "Missing state interface storage for '" +
                full_name + "'");
      }

      interfaces.emplace_back(
        joint.name,
        interface.name,
        &value_it->second);
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
      const auto full_name =
        make_interface_name(joint.name, interface.name);

      auto value_it = command_values_.find(full_name);
      if (value_it == command_values_.end())
      {
        throw std::runtime_error(
                "Missing command interface storage for '" +
                full_name + "'");
      }

      interfaces.emplace_back(
        joint.name,
        interface.name,
        &value_it->second);
    }
  }

  return interfaces;
}

const HumbleSystemInterfaceCompat::HardwareInfoCompat &
HumbleSystemInterfaceCompat::get_hardware_info() const noexcept
{
  return compat_info_;
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

double HumbleSystemInterfaceCompat::get_state(
  const std::string & interface_name) const
{
  return state_values_.at(interface_name);
}

void HumbleSystemInterfaceCompat::set_state(
  const std::string & interface_name,
  const double value)
{
  state_values_.at(interface_name) = value;
}

bool HumbleSystemInterfaceCompat::has_command(
  const std::string & interface_name) const noexcept
{
  return command_values_.find(interface_name) !=
         command_values_.end();
}

double HumbleSystemInterfaceCompat::get_command(
  const std::string & interface_name) const
{
  return command_values_.at(interface_name);
}

void HumbleSystemInterfaceCompat::set_command(
  const std::string & interface_name,
  const double value)
{
  command_values_.at(interface_name) = value;
}

void HumbleSystemInterfaceCompat::shutdown_compat_node() noexcept
{
  stop_executor();
}

std::string HumbleSystemInterfaceCompat::make_interface_name(
  const std::string & component_name,
  const std::string & interface_name)
{
  return component_name + "/" + interface_name;
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
    result = "joint_state_hardware_compat";
  }
  else if (
    std::isdigit(
      static_cast<unsigned char>(result.front())) != 0)
  {
    result.insert(result.begin(), '_');
  }

  return result;
}

double HumbleSystemInterfaceCompat::initial_state_value(
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
  compat_info_.hardware_plugin_name =
    info.hardware_class_type;
  compat_info_.hardware_parameters =
    info.hardware_parameters;
  compat_info_.joints = info.joints;
  compat_info_.mimic_joints.clear();

  // Newer ros2_control provides mimic metadata directly. Humble does not.
  // Recover it from the complete original URDF retained in HardwareInfo.
  urdf::Model robot_model;
  const bool urdf_loaded =
    !info.original_xml.empty() &&
    robot_model.initString(info.original_xml);

  for (std::size_t joint_index = 0;
    joint_index < info.joints.size();
    ++joint_index)
  {
    const auto & joint = info.joints[joint_index];

    bool mimic_found = false;
    std::string mimicked_joint_name;
    double multiplier = 1.0;
    double offset = 0.0;

    if (urdf_loaded)
    {
      const auto urdf_joint =
        robot_model.getJoint(joint.name);

      if (urdf_joint && urdf_joint->mimic)
      {
        mimic_found = true;
        mimicked_joint_name =
          urdf_joint->mimic->joint_name;
        multiplier = urdf_joint->mimic->multiplier;
        offset = urdf_joint->mimic->offset;
      }
    }

    // Also support the package's documented ros2_control per-joint
    // parameters. These take precedence over the regular URDF mimic tag.
    const auto mimic_it =
      joint.parameters.find("mimic");

    if (mimic_it != joint.parameters.end())
    {
      mimic_found = true;
      mimicked_joint_name = mimic_it->second;

      const auto multiplier_it =
        joint.parameters.find("multiplier");

      if (multiplier_it != joint.parameters.end())
      {
        multiplier = std::stod(multiplier_it->second);
      }

      const auto offset_it =
        joint.parameters.find("offset");

      if (offset_it != joint.parameters.end())
      {
        offset = std::stod(offset_it->second);
      }
    }

    if (mimic_found)
    {
      add_mimic_joint(
        joint_index,
        mimicked_joint_name,
        multiplier,
        offset);
    }
  }
}

void HumbleSystemInterfaceCompat::add_mimic_joint(
  const std::size_t joint_index,
  const std::string & mimicked_joint_name,
  const double multiplier,
  const double offset)
{
  const auto mimicked_joint_it = std::find_if(
    compat_info_.joints.begin(),
    compat_info_.joints.end(),
    [&mimicked_joint_name](
      const hardware_interface::ComponentInfo & candidate)
    {
      return candidate.name == mimicked_joint_name;
    });

  if (mimicked_joint_it == compat_info_.joints.end())
  {
    throw std::runtime_error(
            "Mimicked joint '" + mimicked_joint_name +
            "' was not found in ros2_control hardware '" +
            compat_info_.name + "'");
  }

  hardware_interface::MimicJoint mimic_joint;
  mimic_joint.joint_index = joint_index;
  mimic_joint.mimicked_joint_index =
    static_cast<std::size_t>(
    std::distance(
      compat_info_.joints.begin(),
      mimicked_joint_it));
  mimic_joint.multiplier = multiplier;
  mimic_joint.offset = offset;

  compat_info_.mimic_joints.push_back(mimic_joint);
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
      const auto full_name =
        make_interface_name(joint.name, interface.name);

      const auto inserted = state_values_.emplace(
        full_name,
        initial_state_value(interface));

      if (!inserted.second)
      {
        throw std::runtime_error(
                "Duplicate state interface '" +
                full_name + "'");
      }
    }

    for (const auto & interface : joint.command_interfaces)
    {
      const auto full_name =
        make_interface_name(joint.name, interface.name);

      // Match ros2_control initialization behavior: command interfaces begin
      // unset, even when the related state interface has an initial value.
      const auto inserted = command_values_.emplace(
        full_name,
        std::numeric_limits<double>::quiet_NaN());

      if (!inserted.second)
      {
        throw std::runtime_error(
                "Duplicate command interface '" +
                full_name + "'");
      }
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
            "joint_state_humble_system_interface_compat"),
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

}  // namespace joint_state_topic_hardware_interface
