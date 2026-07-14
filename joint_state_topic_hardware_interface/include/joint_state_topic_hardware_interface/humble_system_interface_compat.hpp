// Copyright 2026
// Licensed under the Apache License, Version 2.0

#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

// MimicJoint is part of newer ros2_control hardware metadata, but it does not
// exist in the Humble HardwareInfo API. Define the missing metadata type so the
// upstream JointStateTopicSystem source can remain unchanged.
namespace hardware_interface
{

struct MimicJoint
{
  std::size_t joint_index{};
  std::size_t mimicked_joint_index{};
  double multiplier{1.0};
  double offset{0.0};
};

}  // namespace hardware_interface

namespace joint_state_topic_hardware_interface
{

/**
 * Compatibility base exposing the subset of the newer ros2_control hardware
 * component API used by JointStateTopicSystem while retaining Humble's
 * SystemInterface ABI.
 *
 * The derived source can continue to use:
 *   get_hardware_info()
 *   get_node()
 *   has_state(), get_state(), set_state()
 *   has_command(), get_command(), set_command()
 *
 * This class owns and exports the Humble state and command interface storage.
 */
class HumbleSystemInterfaceCompat :
  public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;

  struct HardwareInfoCompat
  {
    std::string name;
    std::string type;
    std::string hardware_plugin_name;
    std::unordered_map<std::string, std::string>
      hardware_parameters;
    std::vector<hardware_interface::ComponentInfo> joints;
    std::vector<hardware_interface::MimicJoint> mimic_joints;
  };

  HumbleSystemInterfaceCompat() = default;
  ~HumbleSystemInterfaceCompat() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

protected:
  const HardwareInfoCompat &
  get_hardware_info() const noexcept;

  rclcpp::Node::SharedPtr get_node() const noexcept;

  bool has_state(
    const std::string & interface_name) const noexcept;

  double get_state(
    const std::string & interface_name) const;

  void set_state(
    const std::string & interface_name,
    double value);

  bool has_command(
    const std::string & interface_name) const noexcept;

  double get_command(
    const std::string & interface_name) const;

  void set_command(
    const std::string & interface_name,
    double value);

  // Stop callbacks before derived subscriber/publisher members are destroyed.
  // This operation is idempotent.
  void shutdown_compat_node() noexcept;

private:
  static std::string make_interface_name(
    const std::string & component_name,
    const std::string & interface_name);

  static std::string sanitize_node_name(
    const std::string & value);

  static double initial_state_value(
    const hardware_interface::InterfaceInfo & interface);

  void build_compat_hardware_info(
    const hardware_interface::HardwareInfo & info);

  void add_mimic_joint(
    std::size_t joint_index,
    const std::string & mimicked_joint_name,
    double multiplier,
    double offset);

  void initialize_interface_storage(
    const hardware_interface::HardwareInfo & info);

  void start_executor();
  void stop_executor() noexcept;

  HardwareInfoCompat compat_info_;

  // std::map keeps element addresses stable after insertion. Humble interface
  // handles retain raw pointers to these values.
  std::map<std::string, double> state_values_;
  std::map<std::string, double> command_values_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<
    rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

}  // namespace joint_state_topic_hardware_interface
