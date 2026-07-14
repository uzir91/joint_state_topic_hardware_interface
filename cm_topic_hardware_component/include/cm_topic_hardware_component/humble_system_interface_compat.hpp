// Copyright 2026
// Licensed under the Apache License, Version 2.0

#pragma once

#include <cstddef>
#include <cstdint>
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

namespace cm_topic_hardware_component
{

/**
 * Minimal datatype compatibility for the newer ros2_control handle API used by
 * CMTopicSystem. Humble still stores every exported interface as a double.
 */
enum class HandleDataType
{
  DOUBLE,
  BOOL,
  UNSUPPORTED,
};

/**
 * Small metadata handle returned by get_state_interface_handle().
 *
 * This is not the Humble exported StateInterface itself. It only preserves the
 * newer helper API used by the upstream CMTopicSystem implementation.
 */
class HumbleStateInterfaceHandle
{
public:
  HumbleStateInterfaceHandle(std::string name, HandleDataType data_type);

  const std::string & get_name() const noexcept;
  HandleDataType get_data_type() const noexcept;

private:
  std::string name_;
  HandleDataType data_type_{HandleDataType::UNSUPPORTED};
};

/**
 * Compatibility base exposing the subset of the newer ros2_control
 * SystemInterface helper API used by CMTopicSystem, while keeping Humble's ABI.
 *
 * The derived source can keep using:
 *   get_node()
 *   has_state()
 *   get_state_interface_handle()
 *   set_state()
 *
 * This class owns and exports state/command storage for joints, sensors and
 * GPIO components.
 */
class HumbleSystemInterfaceCompat : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  using StateInterfaceHandle =
    std::shared_ptr<const HumbleStateInterfaceHandle>;

  HumbleSystemInterfaceCompat() = default;
  ~HumbleSystemInterfaceCompat() override;

  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

protected:
  rclcpp::Node::SharedPtr get_node() const noexcept;

  bool has_state(
    const std::string & interface_name) const noexcept;

  StateInterfaceHandle get_state_interface_handle(
    const std::string & interface_name) const;

  void set_state(
    const StateInterfaceHandle & handle,
    double value,
    bool force_cast);

  void set_state(
    const StateInterfaceHandle & handle,
    bool value,
    bool force_cast);

  // Must be called by the derived destructor before its subscriber/callback
  // members are destroyed. The operation is idempotent.
  void shutdown_compat_node() noexcept;

private:
  struct InterfaceStorage
  {
    std::string component_name;
    std::string interface_name;
    std::string full_name;
    std::string data_type;
    double value{};
  };

  static std::string sanitize_node_name(
    const std::string & value);

  static HandleDataType parse_data_type(
    const std::string & data_type);

  static double initial_interface_value(
    const hardware_interface::InterfaceInfo & interface);

  void append_component_interfaces(
    const std::vector<hardware_interface::ComponentInfo> & components);

  void start_executor();
  void stop_executor() noexcept;

  // std::map keeps value addresses stable for Humble's exported handles.
  std::map<std::string, double> state_values_;
  std::map<std::string, double> command_values_;

  std::vector<InterfaceStorage> state_interfaces_;
  std::vector<InterfaceStorage> command_interfaces_;

  std::unordered_map<std::string, StateInterfaceHandle>
    state_handles_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor>
    executor_;
  std::thread executor_thread_;
};

}  // namespace cm_topic_hardware_component
