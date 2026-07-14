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
#include <set>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <joint_state_topic_hardware_interface/joint_state_topic_hardware_interface.hpp>
#include <rclcpp/executors.hpp>

namespace
{
/** @brief Sums the total rotation for joint states that wrap from 2*pi to -2*pi
when rotating in the positive direction */
double sumRotationFromMinus2PiTo2Pi(const double current_wrapped_rad, double total_rotation_in)
{
  double delta = 0;
  angles::shortest_angular_distance_with_large_limits(total_rotation_in, current_wrapped_rad, 2 * M_PI, -2 * M_PI,
                                                      delta);

  // Add the corrected delta to the total rotation
  return total_rotation_in + delta;
}
}  // namespace

namespace joint_state_topic_hardware_interface
{

static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;
static constexpr std::size_t VELOCITY_INTERFACE_INDEX = 1;
// JointState doesn't contain an acceleration field, so right now it's not used
static constexpr std::size_t EFFORT_INTERFACE_INDEX = 3;

JointStateTopicSystem::~JointStateTopicSystem()
{
  shutdown_compat_node();
}

CallbackReturn JointStateTopicSystem::on_init(const hardware_interface::HardwareInfo& params)
{
  if (HumbleSystemInterfaceCompat::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = get_hardware_info().hardware_parameters.find(parameter_name);
        it != get_hardware_info().hardware_parameters.end())
    {
      return it->second;
    }
    return default_value;
  };

  if (auto it = get_hardware_info().hardware_parameters.find("trigger_joint_command_threshold");
      it != get_hardware_info().hardware_parameters.end())
  {
    trigger_joint_command_threshold_ = std::stod(it->second);
  }

  topic_based_joint_commands_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"), rclcpp::QoS(1));
  topic_based_joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_states_topic", "/robot_joint_states"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });

  // if the values on the `joint_states_topic` are wrapped between -2*pi and 2*pi (like they are in Isaac Sim)
  // sum the total joint rotation returned on the `joint_state_values_` interface
  if (get_hardware_parameter("sum_wrapped_joint_states", "false") == "true")
  {
    sum_wrapped_joint_states_ = true;
  }

  // TODO(anyone): Remove in a future release after users have migrated to the new plugin name
  if (get_hardware_info().hardware_plugin_name.find("topic_based_ros2_control") != std::string::npos)
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "Plugin name '%s' is deprecated, upgrade to "
                "'joint_state_topic_hardware_interface/JointStateTopicSystem' from package"
                " 'joint_state_topic_hardware_interface' instead.",
                get_hardware_info().hardware_plugin_name.c_str());
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type JointStateTopicSystem::read(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period)
{
  // Copy the latest message while protected by the callback mutex.
  // Do not keep the mutex locked during the entire read operation.
  sensor_msgs::msg::JointState joint_state;
  {
    std::lock_guard<std::mutex> lock(latest_joint_state_mutex_);
    joint_state = latest_joint_state_;
  }

  const auto& hardware_info = get_hardware_info();
  const auto& joints = hardware_info.joints;

  const double dt = period.seconds();
  const bool valid_period = std::isfinite(dt) && dt > 0.0;

  for (std::size_t i = 0; i < joint_state.name.size(); ++i)
  {
    const auto joint_it = std::find_if(
      joints.begin(),
      joints.end(),
      [&joint_state, i](const hardware_interface::ComponentInfo& joint) {
        return joint.name == joint_state.name[i];
      });

    if (joint_it == joints.end())
    {
      // Received a joint that does not belong to this hardware component.
      continue;
    }

    const auto joint_index = static_cast<std::size_t>(
      std::distance(joints.begin(), joint_it));

    const bool is_mimic_joint =
      std::find_if(
        hardware_info.mimic_joints.begin(),
        hardware_info.mimic_joints.end(),
        [joint_index](const hardware_interface::MimicJoint& mimic_joint) {
          return mimic_joint.joint_index == joint_index;
        }) != hardware_info.mimic_joints.end();

    if (is_mimic_joint)
    {
      // Mimic states are calculated after normal joints are updated.
      continue;
    }

    const std::string position_interface =
      joint_state.name[i] + "/" +
      hardware_interface::HW_IF_POSITION;

    const std::string velocity_interface =
      joint_state.name[i] + "/" +
      hardware_interface::HW_IF_VELOCITY;

    const std::string effort_interface =
      joint_state.name[i] + "/" +
      hardware_interface::HW_IF_EFFORT;

    const bool has_valid_position =
      i < joint_state.position.size() &&
      std::isfinite(joint_state.position[i]);

    const bool has_valid_velocity =
      i < joint_state.velocity.size() &&
      std::isfinite(joint_state.velocity[i]);

    const bool has_valid_effort =
      i < joint_state.effort.size() &&
      std::isfinite(joint_state.effort[i]);

    /*
     * Position handling:
     *
     * 1. Use reported position when it is available and finite.
     * 2. Otherwise, integrate velocity to create accumulated position.
     */
    if (has_valid_position && has_state(position_interface))
    {
      if (sum_wrapped_joint_states_)
      {
        double previous_position = get_state(position_interface);

        if (!std::isfinite(previous_position))
        {
          // No previous accumulated position exists yet.
          previous_position = joint_state.position[i];
        }

        set_state(
          position_interface,
          sumRotationFromMinus2PiTo2Pi(
            joint_state.position[i],
            previous_position));
      }
      else
      {
        set_state(
          position_interface,
          joint_state.position[i]);
      }
    }
    else if (
      !has_valid_position &&
      has_valid_velocity &&
      valid_period &&
      has_state(position_interface))
    {
      double position = get_state(position_interface);

      if (!std::isfinite(position))
      {
        position = 0.0;
      }

      position += joint_state.velocity[i] * dt;

      set_state(
        position_interface,
        position);
    }

    // Update the velocity state interface when valid velocity is available.
    if (
      has_valid_velocity &&
      has_state(velocity_interface))
    {
      set_state(
        velocity_interface,
        joint_state.velocity[i]);
    }

    // Update the effort state interface when valid effort is available.
    if (
      has_valid_effort &&
      has_state(effort_interface))
    {
      set_state(
        effort_interface,
        joint_state.effort[i]);
    }
  }

  // Update mimic joint states after their source joints have been updated.
  for (const auto& mimic_joint : hardware_info.mimic_joints)
  {
    const auto& mimic_joint_name =
      joints.at(mimic_joint.joint_index).name;

    const auto& mimicked_joint_name =
      joints.at(mimic_joint.mimicked_joint_index).name;

    const std::string mimic_position =
      mimic_joint_name + "/" +
      hardware_interface::HW_IF_POSITION;

    const std::string source_position =
      mimicked_joint_name + "/" +
      hardware_interface::HW_IF_POSITION;

    if (
      has_state(mimic_position) &&
      has_state(source_position))
    {
      const double source_value = get_state(source_position);

      if (std::isfinite(source_value))
      {
        set_state(
          mimic_position,
          mimic_joint.offset +
          mimic_joint.multiplier * source_value);
      }
    }

    const std::string mimic_velocity =
      mimic_joint_name + "/" +
      hardware_interface::HW_IF_VELOCITY;

    const std::string source_velocity =
      mimicked_joint_name + "/" +
      hardware_interface::HW_IF_VELOCITY;

    if (
      has_state(mimic_velocity) &&
      has_state(source_velocity))
    {
      const double source_value = get_state(source_velocity);

      if (std::isfinite(source_value))
      {
        set_state(
          mimic_velocity,
          mimic_joint.multiplier * source_value);
      }
    }

    const std::string mimic_acceleration =
      mimic_joint_name + "/" +
      hardware_interface::HW_IF_ACCELERATION;

    const std::string source_acceleration =
      mimicked_joint_name + "/" +
      hardware_interface::HW_IF_ACCELERATION;

    if (
      has_state(mimic_acceleration) &&
      has_state(source_acceleration))
    {
      const double source_value = get_state(source_acceleration);

      if (std::isfinite(source_value))
      {
        set_state(
          mimic_acceleration,
          mimic_joint.multiplier * source_value);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JointStateTopicSystem::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  const auto& joints = get_hardware_info().joints;
  // To avoid spamming TopicBased's joint command topic we check the difference between the joint states and
  // the current joint commands, if it's smaller than a threshold we don't publish it.
  auto diff = 0.0;
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    for (const auto& interface : joints[i].command_interfaces)
    {
      const bool supported_command_interface = interface.name == hardware_interface::HW_IF_POSITION ||
                                               interface.name == hardware_interface::HW_IF_VELOCITY ||
                                               interface.name == hardware_interface::HW_IF_EFFORT;
      if (!supported_command_interface)
      {
        continue;
      }
      // sum the absolute difference for all joints
      diff += std::abs(get_state(joints[i].name + "/" + interface.name) -
                       get_command(joints[i].name + "/" + interface.name));
    }
  }
  if (diff <= trigger_joint_command_threshold_)
  {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState joint_state;
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    joint_state.name.push_back(joints[i].name);
    joint_state.header.stamp = get_node()->now();
    // only send commands to the interfaces that are defined for this joint
    for (const auto& interface : joints[i].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position.push_back(get_command(joints[i].name + "/" + interface.name));
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity.push_back(get_command(joints[i].name + "/" + interface.name));
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort.push_back(get_command(joints[i].name + "/" + interface.name));
      }
      else
      {
        RCLCPP_WARN_ONCE(get_node()->get_logger(), "Joint '%s' has unsupported command interfaces found: %s.",
                         joints[i].name.c_str(), interface.name.c_str());
      }
    }
  }

  if (rclcpp::ok())
  {
    topic_based_joint_commands_publisher_->publish(joint_state);
  }

  return hardware_interface::return_type::OK;
}
}  // end namespace joint_state_topic_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(joint_state_topic_hardware_interface::JointStateTopicSystem, hardware_interface::SystemInterface)
