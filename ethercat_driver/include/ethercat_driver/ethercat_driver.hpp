// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_

#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <pluginlib/class_loader.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ethercat_driver/visibility_control.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_master.hpp"
#include "yaml-cpp/yaml.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ethercat_driver
{

class EthercatDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  std::vector<std::unordered_map<std::string, std::string>> getEcModuleParam(
    const std::string & urdf,
    const std::string & component_name,
    const std::string & component_type);

  uint16_t getAliasOrDefaultAlias(
    const std::unordered_map<std::string,
    std::string> & slave_parameters);

  virtual CallbackReturn setupMaster();

  CallbackReturn configNetwork();

  /** @brief Load transfer config YAML file
   * One use case is to load transfers for FailSafe Over EtherCAT Safety
   * @param[out] node YAML node containing the transfer configuration root
   * @param[in] path Path to the YAML file, if empty, the file is loaded from the *fsoe_config*
   * or *transfer_config* of the YAML document
   */
  void loadTransferConfigYamlFile(YAML::Node & node, const std::string & path = "");

  /** @brief Get transfer module parameters from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of maps containing transfer module parameters, each map corresponds to a module
   * involved in a transfer
   */
  std::vector<std::unordered_map<std::string, std::string>> getEcTransferModuleParam(
    const YAML::Node & config);

  /** @brief Get transfer nets from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of transfer nets
   */
  std::vector<ethercat_interface::EcTransferNet> getEcTransferNets(const YAML::Node & config);

  /** @brief Configure the transfer networks
   */
  void configTransferNetwork();

protected:
  // Keep class loader before plugin instances so instances are destroyed first.
  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{
    "ethercat_interface", "ethercat_interface::EcSlave"};

  std::vector<std::shared_ptr<ethercat_interface::EcSlave>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> hw_sensor_commands_;
  std::vector<std::vector<double>> hw_gpio_commands_;
  std::vector<std::vector<double>> hw_joint_states_;
  std::vector<std::vector<double>> hw_sensor_states_;
  std::vector<std::vector<double>> hw_gpio_states_;

  double control_frequency_;

  /** @brief Maximum time (seconds) to wait in on_activate() for all modules to
   * become fully ready (bus-OP + CiA402 OPERATION_ENABLED + valid position).
   * On timeout on_activate() aborts with CallbackReturn::ERROR.
   * Configurable via hardware parameter "init_timeout"; default 30 s. */
  double init_timeout_ = 30.0;
  /** @brief Number of consecutive control cycles for which the full readiness
   * condition must hold before the system is declared started. Guards against
   * transient single-cycle readiness. Hardware parameter "init_stable_cycles";
   * default 10. */
  int init_stable_cycles_ = 10;

  std::shared_ptr<ethercat_interface::EcMaster> master_;
  std::mutex ec_mutex_;
  bool activated_;

  // Safety: tolerate short WC glitches, fail only on persistent non-COMPLETE domain state
  static constexpr int kMaxConsecutiveWcFailures = 5;
  int consecutive_wc_failures_ = 0;
  std::atomic_bool shutdown_requested_{false};

  // --- Deterministic group-barrier startup & runtime group failfast ---
  /** Startup mode: false = legacy (per-drive auto transitions, unchanged
   *  behaviour); true = deterministic phase-0 barrier where all drives are
   *  held at SWITCH_ON_DISABLED until the whole bus is WC_COMPLETE. Hardware parameter
   *  "startup_mode" ("legacy"|"barrier"); default barrier. Set startup_mode
   *  to "legacy" to opt out. */
  bool startup_barrier_mode_ = true;
  /** Maximum time (seconds) allowed for a single barrier phase before the
   *  activation aborts. Hardware parameter "phase_timeout"; default 20 s. */
  double phase_timeout_ = 20.0;
  /** Number of consecutive cycles all drives must hold a barrier phase target
   *  (with WC COMPLETE) before the group advances. Hardware parameter
   *  "phase_stable_cycles"; default 20. */
  int phase_stable_cycles_ = 20;
  /** When true, read() supervises every CiA402 drive each cycle and triggers a
   *  group stop if any drive leaves OPERATION_ENABLED / loses a valid position.
   *  Hardware parameter "runtime_drive_supervision"; default true. Set to
   *  "false" to opt out. */
  bool runtime_drive_supervision_ = true;
  /** Number of consecutive cycles a drive fault must persist before read()
   *  triggers the group stop (debounces transient glitches). Hardware
   *  parameter "runtime_drive_fault_cycles"; default 10. */
  int runtime_drive_fault_cycles_ = 10;
  int consecutive_drive_failures_ = 0;

  /** Run the deterministic startup gate. Returns SUCCESS once all drives are
   *  synchronized at phase-0 (SWITCH_ON_DISABLED), ERROR on phase timeout. */
  CallbackReturn runBarrierStartup();

  /** Transfer nets */
  std::vector<ethercat_interface::EcTransferNet> ec_transfer_nets_;

  /** Indexes of modules inside ec_modules_ vector that are transfer masters */
  std::vector<size_t> ec_transfer_masters_;
  /** Indexes of modules inside ec_modules_ vector that are transfer slaves only */
  std::vector<size_t> ec_transfer_slaves_;

  /** Empty interfaces */
  std::vector<double> empty_interface_;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
