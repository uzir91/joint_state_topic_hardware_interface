// Copyright 2025 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Jafar Abdi */

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

#include <angles/angles.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <joint_state_topic_hardware_interface/joint_state_topic_hardware_interface.hpp>

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

double sum_rotation_from_minus_2_pi_to_2_pi(
  const double current_wrapped_rad,
  const double total_rotation_in)
{
  double delta = 0.0;
  angles::shortest_angular_distance_with_large_limits(
    total_rotation_in,
    current_wrapped_rad,
    -2.0 * kPi,
    2.0 * kPi,
    delta);

  return total_rotation_in + delta;
}

}  // namespace

namespace joint_state_topic_hardware_interface
{

JointStateTopicSystem::~JointStateTopicSystem()
{
  shutdown_compat_node();
}

CallbackReturn JointStateTopicSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (HumbleSystemInterfaceCompat::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto get_hardware_parameter =
    [this](const std::string & parameter_name, const std::string & default_value) {
      const auto & parameters = get_hardware_info().hardware_parameters;
      const auto parameter_it = parameters.find(parameter_name);
      return parameter_it == parameters.end() ? default_value : parameter_it->second;
    };

  const auto threshold_it =
    get_hardware_info().hardware_parameters.find("trigger_joint_command_threshold");
  if (threshold_it != get_hardware_info().hardware_parameters.end())
  {
    trigger_joint_command_threshold_ = std::stod(threshold_it->second);
  }

  topic_based_joint_commands_publisher_ =
    get_node()->create_publisher<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"),
      rclcpp::QoS(1));

  topic_based_joint_states_subscriber_ =
    get_node()->create_subscription<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_states_topic", "/robot_joint_states"),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr joint_state) {
        std::lock_guard<std::mutex> lock(latest_joint_state_mutex_);
        latest_joint_state_ = std::move(*joint_state);
      });

  sum_wrapped_joint_states_ =
    get_hardware_parameter("sum_wrapped_joint_states", "false") == "true";

  if (get_hardware_info().hardware_plugin_name.find("topic_based_ros2_control") !=
      std::string::npos)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Plugin name '%s' is deprecated; use "
      "'joint_state_topic_hardware_interface/JointStateTopicSystem'.",
      get_hardware_info().hardware_plugin_name.c_str());
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type JointStateTopicSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::JointState joint_state;
  {
    std::lock_guard<std::mutex> lock(latest_joint_state_mutex_);
    joint_state = latest_joint_state_;
  }

  const auto & joints = get_hardware_info().joints;

  for (std::size_t message_index = 0;
       message_index < joint_state.name.size();
       ++message_index)
  {
    const auto joint_it = std::find_if(
      joints.begin(), joints.end(),
      [&joint_state, message_index](const hardware_interface::ComponentInfo & joint) {
        return joint.name == joint_state.name[message_index];
      });

    if (joint_it == joints.end())
    {
      continue;
    }

    const auto joint_index = static_cast<std::size_t>(
      std::distance(joints.begin(), joint_it));

    const auto mimic_it = std::find_if(
      get_hardware_info().mimic_joints.begin(),
      get_hardware_info().mimic_joints.end(),
      [joint_index](const auto & mimic_joint) {
        return mimic_joint.joint_index == joint_index;
      });

    if (mimic_it != get_hardware_info().mimic_joints.end())
    {
      continue;
    }

    const auto & joint_name = joint_state.name[message_index];

    if (message_index < joint_state.position.size() &&
        std::isfinite(joint_state.position[message_index]))
    {
      const auto interface_name =
        joint_name + "/" + hardware_interface::HW_IF_POSITION;

      if (has_state(interface_name))
      {
        if (sum_wrapped_joint_states_)
        {
          const auto previous_position = get_state(interface_name);
          set_state(
            interface_name,
            std::isfinite(previous_position) ?
            sum_rotation_from_minus_2_pi_to_2_pi(
              joint_state.position[message_index], previous_position) :
            joint_state.position[message_index]);
        }
        else
        {
          set_state(interface_name, joint_state.position[message_index]);
        }
      }
    }

    if (message_index < joint_state.velocity.size() &&
        std::isfinite(joint_state.velocity[message_index]))
    {
      const auto interface_name =
        joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
      if (has_state(interface_name))
      {
        set_state(interface_name, joint_state.velocity[message_index]);
      }
    }

    if (message_index < joint_state.effort.size() &&
        std::isfinite(joint_state.effort[message_index]))
    {
      const auto interface_name =
        joint_name + "/" + hardware_interface::HW_IF_EFFORT;
      if (has_state(interface_name))
      {
        set_state(interface_name, joint_state.effort[message_index]);
      }
    }
  }

  for (const auto & mimic_joint : get_hardware_info().mimic_joints)
  {
    const auto & mimic_joint_name = joints.at(mimic_joint.joint_index).name;
    const auto & mimicked_joint_name =
      joints.at(mimic_joint.mimicked_joint_index).name;

    const auto mimic_position =
      mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const auto source_position =
      mimicked_joint_name + "/" + hardware_interface::HW_IF_POSITION;
    if (has_state(mimic_position) && has_state(source_position))
    {
      set_state(
        mimic_position,
        mimic_joint.offset + mimic_joint.multiplier * get_state(source_position));
    }

    const auto mimic_velocity =
      mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto source_velocity =
      mimicked_joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    if (has_state(mimic_velocity) && has_state(source_velocity))
    {
      set_state(
        mimic_velocity,
        mimic_joint.multiplier * get_state(source_velocity));
    }

    const auto mimic_acceleration =
      mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const auto source_acceleration =
      mimicked_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION;
    if (has_state(mimic_acceleration) && has_state(source_acceleration))
    {
      set_state(
        mimic_acceleration,
        mimic_joint.multiplier * get_state(source_acceleration));
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JointStateTopicSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  const auto & joints = get_hardware_info().joints;

  double difference = 0.0;
  for (const auto & joint : joints)
  {
    for (const auto & interface : joint.command_interfaces)
    {
      const bool supported =
        interface.name == hardware_interface::HW_IF_POSITION ||
        interface.name == hardware_interface::HW_IF_VELOCITY ||
        interface.name == hardware_interface::HW_IF_EFFORT;

      if (!supported)
      {
        continue;
      }

      const auto full_name = joint.name + "/" + interface.name;
      if (!has_state(full_name) || !has_command(full_name))
      {
        difference = std::numeric_limits<double>::infinity();
        continue;
      }

      const auto state = get_state(full_name);
      const auto command = get_command(full_name);
      if (!std::isfinite(state) || !std::isfinite(command))
      {
        difference = std::numeric_limits<double>::infinity();
        continue;
      }

      difference += std::abs(state - command);
    }
  }

  if (difference <= trigger_joint_command_threshold_)
  {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState command_message;
  command_message.header.stamp = get_node()->now();

  bool has_position_commands = false;
  bool has_velocity_commands = false;
  bool has_effort_commands = false;

  for (const auto & joint : joints)
  {
    has_position_commands = has_position_commands ||
      has_command(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    has_velocity_commands = has_velocity_commands ||
      has_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
    has_effort_commands = has_effort_commands ||
      has_command(joint.name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  for (const auto & joint : joints)
  {
    command_message.name.push_back(joint.name);

    if (has_position_commands)
    {
      const auto full_name =
        joint.name + "/" + hardware_interface::HW_IF_POSITION;
      command_message.position.push_back(
        has_command(full_name) ? get_command(full_name) :
        std::numeric_limits<double>::quiet_NaN());
    }

    if (has_velocity_commands)
    {
      const auto full_name =
        joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      command_message.velocity.push_back(
        has_command(full_name) ? get_command(full_name) :
        std::numeric_limits<double>::quiet_NaN());
    }

    if (has_effort_commands)
    {
      const auto full_name =
        joint.name + "/" + hardware_interface::HW_IF_EFFORT;
      command_message.effort.push_back(
        has_command(full_name) ? get_command(full_name) :
        std::numeric_limits<double>::quiet_NaN());
    }

    for (const auto & interface : joint.command_interfaces)
    {
      const bool supported =
        interface.name == hardware_interface::HW_IF_POSITION ||
        interface.name == hardware_interface::HW_IF_VELOCITY ||
        interface.name == hardware_interface::HW_IF_EFFORT;
      if (!supported)
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Joint '%s' has unsupported command interface '%s'.",
          joint.name.c_str(), interface.name.c_str());
      }
    }
  }

  if (rclcpp::ok())
  {
    topic_based_joint_commands_publisher_->publish(command_message);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace joint_state_topic_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  joint_state_topic_hardware_interface::JointStateTopicSystem,
  hardware_interface::SystemInterface)