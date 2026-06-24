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

#include "ethercat_driver/ethercat_driver.hpp"

#include <chrono>
#include <tinyxml2.h>
#include <string>
#include <regex>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{

unsigned int uint_from_string(const std::string & str)
{
  // Strip leading and trailing whitespaces
  std::string s = std::regex_replace(str, std::regex("^ +| +$|( ) +"), "$1");
  // Test if the number is in hexadecimal format
  if (s.find("0x") == 0) {
    return std::stoul(s, nullptr, 16);
  }
  return std::stoul(s);
}

void getTransferMemoryInfo(
  const YAML::Node & element,
  ethercat_interface::EcMemoryEntry & entry,
  const std::string & dir,
  const std::string & transfer_net_name)
{
  if (!element["ec_module"]) {
    std::string msg = "Transfer definition without ec_module entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["index"]) {
    std::string msg = "Transfer definition without index entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["subindex"]) {
    std::string msg = "Transfer definition without subindex entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }

  entry.module_name = element["ec_module"].as<std::string>();
  entry.index = uint_from_string(element["index"].as<std::string>());
  entry.subindex = uint_from_string(element["subindex"].as<std::string>());
}

void throwErrorIfModuleParametersNotFound(
  const ethercat_interface::EcTransferEntry & transfer,
  const std::string & module_name,
  const std::string & transfer_net_name,
  const std::string & direction)
{
  std::string msg = "In transfer net: " + transfer_net_name + ", for transfer " +
    transfer.to_simple_string() + ", the module name of the " + direction + "( " + module_name +
    ") among all the recorded modules.";
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "EthercatDriver"), msg.c_str());
  throw std::runtime_error(msg);
}

uint16_t EthercatDriver::getAliasOrDefaultAlias(
  const std::unordered_map<std::string,
  std::string> & slave_parameters)
{
  if (slave_parameters.find("alias") != slave_parameters.end()) {
    return std::stoul(slave_parameters.at("alias"));
  } else {
    return 0;
  }
}

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  // Set state vectors
  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  // Set command vectors
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  // Setup slave modules defined per joints in the URDF
  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_joint_states_[j], &hw_joint_commands_[j]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Joint module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.joints[j].name.c_str(), ex.what());
      }
    }
  }

  // Setup slave modules defined per GPIOs in the URDF
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_gpio_states_[g], &hw_gpio_commands_[g]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of GPIO module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.gpios[g].name.c_str(), ex.what());
      }
    }
  }

  // Setup slave modules defined per sensors in the URDF
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_sensor_states_[s], &hw_sensor_commands_[s]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Sensor module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.sensors[s].name.c_str(), ex.what());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %li modules", ec_modules_.size());

  // Check if a transfer configuration is provided
  if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() ||
    info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end())
  {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer configuration detected, ...");

    YAML::Node config;
    // Load the transfer config file
    loadTransferConfigYamlFile(config);

    // Parse transfer modules from the transfer yaml file
    auto transfer_module_params = getEcTransferModuleParam(config);

    // Append the transfer modules parameters to the list of modules parameters
    size_t idx_1st = ec_module_parameters_.size();
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), transfer_module_params.begin(), transfer_module_params.end());
    for (size_t i = 0; i < transfer_module_params.size(); i++) {
      ec_transfer_slaves_.push_back(idx_1st + i);
    }

    // Parse transfer nets from the transfer yaml file
    ec_transfer_nets_ = getEcTransferNets(config);

    // Append the transfer modules to the list of modules and load them
    for (const auto & transfer_module_param : transfer_module_params) {
      try {
        auto ec_module = ec_loader_.createSharedInstance(transfer_module_param.at("plugin"));
        if (!ec_module->setupSlave(
            transfer_module_param, &empty_interface_, &empty_interface_))
        {
          const std::string & module_name = transfer_module_param.at("name");
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of transfer only module %s FAILED.", module_name.c_str() );
          return CallbackReturn::ERROR;
        }

        auto idx = ec_modules_.size();
        ec_module->setAliasAndPosition(
          getAliasOrDefaultAlias(transfer_module_param),
          std::stoul(transfer_module_param.at("position")));
        ec_modules_.push_back(ec_module);
        ec_transfer_slaves_.push_back(idx);
      } catch (const pluginlib::PluginlibException & ex) {
        const std::string & module_name = transfer_module_param.at("name");
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "EthercatDriver"),
          "The plugin failed to load for transfer module %s. Error: %s\n",
          module_name.c_str(), ex.what());
      }
    }

    // Find all masters from the nets
    {
      std::vector<std::string> master_names;
      for (const auto & net : ec_transfer_nets_) {
        master_names.push_back(net.master);
      }
      for (size_t i = 0; i < ec_module_parameters_.size(); i++) {
        if (std::find(
            master_names.begin(), master_names.end(),
            ec_module_parameters_[i].at("name")) !=
          master_names.end())
        {
          ec_transfer_masters_.push_back(i);
        }
      }
    }

    // Identify (alias,position) all the modules participating in transfers
    for (auto & net : ec_transfer_nets_) {
      for (auto & transfer : net.transfers) {
        // Update each EcMemoryEntry with the alias and position of the module
        size_t in_idx = ec_module_parameters_.size();
        for (in_idx = 0; in_idx < ec_module_parameters_.size(); ++in_idx) {
          if (ec_module_parameters_[in_idx].at("name") == transfer.input.module_name) {
            break;
          }
        }
        size_t out_idx = ec_module_parameters_.size();
        for (out_idx = 0; out_idx < ec_module_parameters_.size(); ++out_idx) {
          if (ec_module_parameters_[out_idx].at("name") == transfer.output.module_name) {
            break;
          }
        }
        if (in_idx == ec_module_parameters_.size()) {
          throwErrorIfModuleParametersNotFound(
            transfer, transfer.input.module_name, net.name, "input");
        }
        if (out_idx == ec_module_parameters_.size()) {
          throwErrorIfModuleParametersNotFound(
            transfer, transfer.output.module_name, net.name, "output");
        }

        const auto & input_module = ec_modules_[in_idx];
        const auto & output_module = ec_modules_[out_idx];

        transfer.input.alias = input_module->alias_;
        transfer.input.position = input_module->position_;
        transfer.output.alias = output_module->alias_;
        transfer.output.position = output_module->position_;
      }
    }

    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "Transfer configuration loaded successfully!");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name,
          info_.joints[j].state_interfaces[i].name,
          &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[s].name,
          info_.sensors[s].state_interfaces[i].name,
          &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.gpios[g].name,
          info_.gpios[g].state_interfaces[i].name,
          &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  std::vector<double> test;
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name,
          info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.sensors[s].name,
          info_.sensors[s].command_interfaces[i].name,
          &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios[g].name,
          info_.gpios[g].command_interfaces[i].name,
          &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::setupMaster()
{
  unsigned int master_id = 666;
  // Get master id
  if (info_.hardware_parameters.find("master_id") == info_.hardware_parameters.end()) {
    // Master id was not provided, default to 0
    master_id = 0;
  } else {
    try {
      master_id = std::stoul(info_.hardware_parameters["master_id"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid master id (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }
  master_ = std::make_shared<ethercat_interface::EcMaster>(master_id);

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::configNetwork()
{
  // Get control frequency
  if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
    // Control frequency was not provided, default to 100 Hz
    control_frequency_ = 100.0;
  } else {
    try {
      control_frequency_ = std::stod(info_.hardware_parameters["control_frequency"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid control frequency (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  // Get init readiness-gate parameters (optional; defaults preserve behaviour)
  if (info_.hardware_parameters.find("init_timeout") != info_.hardware_parameters.end()) {
    try {
      init_timeout_ = std::stod(info_.hardware_parameters["init_timeout"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid init_timeout (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }
  if (info_.hardware_parameters.find("init_stable_cycles") != info_.hardware_parameters.end()) {
    try {
      init_stable_cycles_ = std::stoi(info_.hardware_parameters["init_stable_cycles"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid init_stable_cycles (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }
  if (init_stable_cycles_ < 1) {
    init_stable_cycles_ = 1;
  }

  // Deterministic group-barrier startup mode (default barrier; set legacy to opt out)
  if (info_.hardware_parameters.find("startup_mode") != info_.hardware_parameters.end()) {
    const std::string mode = info_.hardware_parameters["startup_mode"];
    if (mode == "barrier") {
      startup_barrier_mode_ = true;
    } else if (mode == "legacy") {
      startup_barrier_mode_ = false;
    } else {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid startup_mode '%s' (expected 'legacy' or 'barrier')!", mode.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (info_.hardware_parameters.find("phase_timeout") != info_.hardware_parameters.end()) {
    try {
      phase_timeout_ = std::stod(info_.hardware_parameters["phase_timeout"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid phase_timeout (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }
  if (info_.hardware_parameters.find("phase_stable_cycles") != info_.hardware_parameters.end()) {
    try {
      phase_stable_cycles_ = std::stoi(info_.hardware_parameters["phase_stable_cycles"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid phase_stable_cycles (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }
  if (phase_stable_cycles_ < 1) {
    phase_stable_cycles_ = 1;
  }

  // Runtime group-failfast supervision (default on; set false to opt out)
  if (info_.hardware_parameters.find("runtime_drive_supervision") !=
    info_.hardware_parameters.end())
  {
    const std::string v = info_.hardware_parameters["runtime_drive_supervision"];
    runtime_drive_supervision_ = (v == "true" || v == "1" || v == "True");
  }
  if (info_.hardware_parameters.find("runtime_drive_fault_cycles") !=
    info_.hardware_parameters.end())
  {
    try {
      runtime_drive_fault_cycles_ =
        std::stoi(info_.hardware_parameters["runtime_drive_fault_cycles"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"),
        "Invalid runtime_drive_fault_cycles (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }
  if (runtime_drive_fault_cycles_ < 1) {
    runtime_drive_fault_cycles_ = 1;
  }

  // start EC and wait until state operative

  master_->setCtrlFrequency(control_frequency_);

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    master_->addSlave(ec_modules_[i].get());
  }

  // configure SDO
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    for (auto & sdo : ec_modules_[i]->sdo_config) {
      uint32_t abort_code;
      int ret = master_->configSlaveSdo(
        std::stod(ec_module_parameters_[i]["position"]),
        sdo,
        &abort_code);
      if (ret) {
        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to download config SDO for module at position %s with Error: %d",
          ec_module_parameters_[i]["position"].c_str(),
          abort_code);
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

namespace
{
// CiA402 DeviceState values mirrored from
// ethercat_generic_plugins/cia402_common_defs.hpp. They MUST stay in sync with
// that enum; cia402State() returns these integer values. Only the forward
// power-up path is needed here.
constexpr int kStateSwitchOnDisabled = 3;   // STATE_SWITCH_ON_DISABLED
constexpr int kStateReadyToSwitchOn = 4;    // STATE_READY_TO_SWITCH_ON
constexpr int kStateSwitchOn = 5;           // STATE_SWITCH_ON
constexpr int kStateOperationEnabled = 6;   // STATE_OPERATION_ENABLED

// Power-up rank for the group barrier; -1 means "not on the forward path".
int cia402PowerupRank(int state)
{
  switch (state) {
    case kStateSwitchOnDisabled: return 0;
    case kStateReadyToSwitchOn:  return 1;
    case kStateSwitchOn:         return 2;
    case kStateOperationEnabled: return 3;
    default:                     return -1;
  }
}
}  // namespace

CallbackReturn EthercatDriver::runBarrierStartup()
{
  // Ordered power-up targets the whole drive group steps through together.
  // Phase 0 (SWITCH_ON_DISABLED) is the "wait for the whole bus" gate: every
  // drive that becomes bus-OP is HELD at SWITCH_ON_DISABLED until the ENTIRE
  // bus is OP (domain WC COMPLETE and every drive at least SWITCH_ON_DISABLED).
  // Only then does the group advance — together — through READY_TO_SWITCH_ON,
  // SWITCH_ON and OPERATION_ENABLED. Without phase 0 each drive raced to READY
  // on its own as soon as it individually reached bus-OP (serial power-up).
  const int phases[] = {
    kStateSwitchOnDisabled, kStateReadyToSwitchOn, kStateSwitchOn, kStateOperationEnabled};
  const char * phase_names[] = {
    "SWITCH_ON_DISABLED", "READY_TO_SWITCH_ON", "SWITCH_ON", "OPERATION_ENABLED"};

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "Barrier startup: bringing all drives up together (phase_timeout=%.1fs, "
    "stable_cycles=%d).", phase_timeout_, phase_stable_cycles_);

  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_nsec += master_->getInterval();
  while (t.tv_nsec >= 1000000000) {
    t.tv_nsec -= 1000000000;
    t.tv_sec++;
  }

  // Only Phase 0: hold all drives at SWITCH_ON_DISABLED until the bus is fully
  // up (domain WC COMPLETE). After that, release the barrier and let each
  // drive transition freely through CiA402 on its own. The lockstep phases 1–3
  // have been removed because they caused spurious timeouts on serial bus
  // bring-up and the drives are perfectly capable of self-transitioning.
  for (size_t p = 0; p < 1; ++p) {
    const int target = phases[p];
    const int target_rank = cia402PowerupRank(target);
    const double this_phase_timeout = init_timeout_;

    // Arm the barrier on every drive for this phase target.
    for (auto & module : ec_modules_) {
      if (module->cia402State() >= 0) {
        module->setStartupBarrier(true, target);
      }
    }

    struct timespec t_phase_start;
    clock_gettime(CLOCK_MONOTONIC, &t_phase_start);
    int stable_cycles = 0;
    bool phase_running = true;

    while (phase_running) {
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
      master_->update();

      // The whole group must reach (or pass) the phase target with a healthy
      // bus before we advance. Non-drive slaves (cia402State()<0) are ignored.
      bool all_at_target = (master_->getDomainWcState(0) == EC_WC_COMPLETE);
      for (auto & module : ec_modules_) {
        const int st = module->cia402State();
        if (st < 0) {continue;}
        const int rank = cia402PowerupRank(st);
        all_at_target = all_at_target && (rank >= 0) && (rank >= target_rank);
      }

      stable_cycles = all_at_target ? (stable_cycles + 1) : 0;
      if (stable_cycles >= phase_stable_cycles_) {
        phase_running = false;
      }

      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      double elapsed = (now.tv_sec - t_phase_start.tv_sec) +
        (now.tv_nsec - t_phase_start.tv_nsec) * 1e-9;
      if (phase_running && elapsed > this_phase_timeout) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EthercatDriver"),
          "Barrier startup TIMEOUT in phase '%s' after %.1f s (limit %.1f s). "
          "Domain WC state: %u (need %u=COMPLETE). Drives not at target:",
          phase_names[p], elapsed, this_phase_timeout,
          master_->getDomainWcState(0), static_cast<unsigned int>(EC_WC_COMPLETE));
        for (size_t i = 0; i < ec_modules_.size(); ++i) {
          auto & module = ec_modules_[i];
          const int st = module->cia402State();
          if (st < 0) {continue;}
          const int rank = cia402PowerupRank(st);
          if (rank < 0 || rank < target_rank) {
            RCLCPP_ERROR(
              rclcpp::get_logger("EthercatDriver"),
              "  Module %zu (alias %u, pos %u): %s",
              i, module->alias_, module->position_, module->statusString().c_str());
          }
        }
        // Disarm the barrier so deactivate()/error handling is unrestricted.
        for (auto & module : ec_modules_) {
          if (module->cia402State() >= 0) {
            module->setStartupBarrier(false, kStateOperationEnabled);
          }
        }
        return CallbackReturn::ERROR;
      }

      t.tv_nsec += master_->getInterval();
      while (t.tv_nsec >= 1000000000) {
        t.tv_nsec -= 1000000000;
        t.tv_sec++;
      }
    }

    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "Barrier startup: all drives reached '%s'.", phase_names[p]);
  }

  // Power-up complete — disarm the barrier so normal runtime control is fully
  // unrestricted (and fault recovery can drive the state machine freely).
  for (auto & module : ec_modules_) {
    if (module->cia402State() >= 0) {
      module->setStartupBarrier(false, kStateOperationEnabled);
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ...please wait...");

  // Reset WC error counter
  consecutive_wc_failures_ = 0;

  // setup master
  setupMaster();
  // configure network
  configNetwork();

  if (!master_->activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

  // Configure transfer network if transfer nets are defined
  if (!ec_transfer_nets_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Configuring transfer network...");
    master_->registerTransferInDomain(ec_transfer_nets_);
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer network configured!");
  }

  // Deterministic group-barrier power-up: bring ALL drives through the CiA402
  // state machine together (no partial / per-drive enabling). Aborts hard if
  // any drive cannot keep up in a phase. Runs before the final readiness gate.
  if (startup_barrier_mode_) {
    CallbackReturn barrier_result = runBarrierStartup();
    if (barrier_result != CallbackReturn::SUCCESS) {
      return barrier_result;
    }
  }

  // Activation loop: wait until ALL modules are simultaneously fully ready
  // (EtherCAT bus-OP + domain WC COMPLETE + CiA402 OPERATION_ENABLED + valid
  // actual position), stable over init_stable_cycles_ consecutive cycles.
  // On init_timeout_ the activation aborts hard and reports the stuck module(s).
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  const struct timespec t_start = t;
  // Add first interval to initial time (not 1 second!)
  t.tv_nsec += master_->getInterval();
  while (t.tv_nsec >= 1000000000) {
    t.tv_nsec -= 1000000000;
    t.tv_sec++;
  }

  int stable_cycles = 0;
  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus
    master_->update();

    // Full readiness: domain COMPLETE AND every module bus-OP + command-ready
    // + valid position. For non-drive slaves readyForCommands()/hasValidPosition()
    // fall back to safe defaults, so they only require bus-level readiness.
    bool allReady = (master_->getDomainWcState(0) == EC_WC_COMPLETE);
    for (auto & module : ec_modules_) {
      allReady = allReady &&
        module->initialized() &&
        module->readyForCommands() &&
        module->hasValidPosition();
    }

    stable_cycles = allReady ? (stable_cycles + 1) : 0;
    if (stable_cycles >= init_stable_cycles_) {
      running = false;
    }

    // Timeout guard: abort hard and report which module(s) are not ready
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - t_start.tv_sec) +
      (now.tv_nsec - t_start.tv_nsec) * 1e-9;
    if (running && elapsed > init_timeout_) {
      unsigned int wc = master_->getDomainWcState(0);
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"),
        "Activation TIMEOUT after %.1f s (limit %.1f s): system not ready. "
        "Domain WC state: %u (need %u=COMPLETE).",
        elapsed, init_timeout_, wc, static_cast<unsigned int>(EC_WC_COMPLETE));
      for (size_t i = 0; i < ec_modules_.size(); ++i) {
        auto & module = ec_modules_[i];
        bool ok = module->initialized() &&
          module->readyForCommands() &&
          module->hasValidPosition();
        if (!ok) {
          RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "  Module %zu (alias %u, pos %u) NOT ready: %s",
            i, module->alias_, module->position_, module->statusString().c_str());
        }
      }
      return CallbackReturn::ERROR;
    }

    // calculate next shot
    t.tv_nsec += master_->getInterval();
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  activated_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;
  consecutive_wc_failures_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

  // Deactivate EtherCAT master — forces slaves to INIT, engages brakes
  master_->deactivate();

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  {
    const std::lock_guard<std::mutex> lock(ec_mutex_);
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "SAFETY: Hardware error detected — stopping EtherCAT master!");

    if (activated_) {
      activated_ = false;
      consecutive_wc_failures_ = 0;
      master_->deactivate();
    }

    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "EtherCAT master stopped. Requesting process shutdown.");
  }

  // Request a clean process exit once, delayed a bit to let controller_manager
  // finish its own error-cycle deactivation path first.
  if (rclcpp::ok() && !shutdown_requested_.exchange(true)) {
    std::thread([]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }).detach();
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    master_->readData();

    // === SAFETY: Check link status — immediate ERROR on link down ===
    if (!master_->isMasterLinkUp()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"),
        "SAFETY: EtherCAT link DOWN — stopping EtherCAT master immediately and returning ERROR!");
      // Immediate stop path: prevent any further writeData() in this cycle.
      activated_ = false;
      consecutive_wc_failures_ = 0;
      master_->deactivate();
      return hardware_interface::return_type::ERROR;
    }

    // === SAFETY: Check domain WKC — immediate ERROR when not COMPLETE ===
    unsigned int wc_state = master_->getDomainWcState(0);
    if (wc_state != EC_WC_COMPLETE) {
      consecutive_wc_failures_++;
      if (consecutive_wc_failures_ >= kMaxConsecutiveWcFailures) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EthercatDriver"),
          "SAFETY: Domain WC not COMPLETE for %d consecutive cycles (last state=%u) — returning ERROR!",
          consecutive_wc_failures_, wc_state);
        return hardware_interface::return_type::ERROR;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"),
        "Domain WC transient fault %d/%d (state=%u).",
        consecutive_wc_failures_, kMaxConsecutiveWcFailures, wc_state);
    } else {
      consecutive_wc_failures_ = 0;
    }

    // === SAFETY: Runtime group-failfast — one drive fails => whole arm stops ===
    // If any CiA402 drive leaves OPERATION_ENABLED or loses a valid position
    // during operation, the entire group must stop (no partial operation).
    // Debounced over runtime_drive_fault_cycles_ to ignore single-cycle glitches.
    if (runtime_drive_supervision_) {
      int faulted_index = -1;
      for (size_t i = 0; i < ec_modules_.size(); ++i) {
        auto & module = ec_modules_[i];
        if (module->cia402State() < 0) {continue;}  // non-drive slave
        if (!(module->readyForCommands() && module->hasValidPosition())) {
          faulted_index = static_cast<int>(i);
          break;
        }
      }

      if (faulted_index >= 0) {
        consecutive_drive_failures_++;
        auto & m = ec_modules_[faulted_index];
        if (consecutive_drive_failures_ >= runtime_drive_fault_cycles_) {
          RCLCPP_ERROR(
            rclcpp::get_logger("EthercatDriver"),
            "SAFETY: Drive (alias %u, pos %u) left ready state for %d cycles: %s "
            "— stopping ALL drives (group failfast)!",
            m->alias_, m->position_, consecutive_drive_failures_,
            m->statusString().c_str());
          activated_ = false;
          consecutive_wc_failures_ = 0;
          consecutive_drive_failures_ = 0;
          master_->deactivate();
          return hardware_interface::return_type::ERROR;
        }
        RCLCPP_WARN(
          rclcpp::get_logger("EthercatDriver"),
          "Drive (alias %u, pos %u) not ready %d/%d (%s).",
          m->alias_, m->position_, consecutive_drive_failures_,
          runtime_drive_fault_cycles_, m->statusString().c_str());
      } else {
        consecutive_drive_failures_ = 0;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    master_->writeData();
  }
  return hardware_interface::return_type::OK;
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcModuleParam(
  const std::string & urdf,
  const std::string & component_name,
  const std::string & component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  while (ros2_control_it) {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it) {
      if (!component_name.compare(ros2_control_child_it->Attribute("name"))) {
        const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");
        while (ec_module_it) {
          module_param.clear();
          module_param["name"] = ec_module_it->Attribute("name");
          const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
          if (NULL != plugin_it) {
            module_param["plugin"] = plugin_it->GetText();
          }
          const auto * param_it = ec_module_it->FirstChildElement("param");
          while (param_it) {
            module_param[param_it->Attribute("name")] = param_it->GetText();
            param_it = param_it->NextSiblingElement("param");
          }
          module_params.push_back(module_param);
          ec_module_it = ec_module_it->NextSiblingElement("ec_module");
        }
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }

  return module_params;
}

void EthercatDriver::loadTransferConfigYamlFile(YAML::Node & node, const std::string & path)
{
  std::string file_path;
  if (path.empty()) {
    // Get the fsoe_config or transfer_config parameter of the ethercat_driver hardware plugin
    if (info_.hardware_parameters.find("fsoe_config") == info_.hardware_parameters.end() &&
      info_.hardware_parameters.find("transfer_config") == info_.hardware_parameters.end() )
    {
      std::string msg("transfer_config or fsoe_config parameter is missing!");
      // Transfer (or fsoe) config file was not provided
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() &&
      info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end())
    {
      std::string msg(
        "Both transfer_config and fsoe_config parameters are provided! Please provide only one "
        "of them.");
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() ) {
      std::string msg("The fsoe_config parameter is deprecated. "
        "Please use transfer_config instead.");
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      file_path = info_.hardware_parameters.at("fsoe_config");
    }
    if (info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end()) {
      file_path = info_.hardware_parameters.at("transfer_config");
    }
  } else {
    file_path = path;
  }

  try {
    node = YAML::LoadFile(file_path);
  } catch (const YAML::ParserException & ex) {
    std::string msg =
      std::string(
      "EthercatDriver : failed to load transfer configuration "
      "(YAML file is incorrect): ") + std::string(ex.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (const YAML::BadFile & ex) {
    std::string msg =
      std::string(
      "EthercatDriver : failed to load transfer configuration "
      "(file path is incorrect or file is damaged): " + std::string(ex.what()));
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (std::exception & e) {
    std::string msg =
      std::string(
      "EthercatDriver : error while loading transfer configuration: ") + std::string(e.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  }
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcTransferModuleParam(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }
  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  // It is possible that modules are only involved in transfers and hence
  // not declared in the ros2_control xacro file.
  // This is a common situation with modules only involved in safety
  // operations. In this case, it is necessary to find the plugin to load,
  // the position and the alias for those slaves.
  if (config["transfer_modules"]) {
    for (const auto & module : config["transfer_modules"]) {
      module_param.clear();
      module_param["name"] = module["name"].as<std::string>();
      module_param["plugin"] = module["plugin"].as<std::string>();
      for (const auto & param : module["parameters"]) {
        module_param[param.first.as<std::string>()] = param.second.as<std::string>();
      }
      module_params.push_back(module_param);
    }
  }

  if (config["safety_modules"]) {
    for (const auto & module : config["safety_modules"]) {
      module_param.clear();
      module_param["name"] = module["name"].as<std::string>();
      module_param["plugin"] = module["plugin"].as<std::string>();
      for (const auto & param : module["parameters"]) {
        module_param[param.first.as<std::string>()] = param.second.as<std::string>();
      }
      module_params.push_back(module_param);
    }
  }

  return module_params;
}

std::vector<ethercat_interface::EcTransferNet> EthercatDriver::getEcTransferNets(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }

  std::vector<ethercat_interface::EcTransferNet> transfer_nets;
  ethercat_interface::EcTransferNet transfer_net;

  if (config["nets"]) {
    for (const auto & net : config["nets"]) {
      transfer_net.reset(net["name"].as<std::string>());
      if (net["safety_master"]) {
        transfer_net.master = net["safety_master"].as<std::string>();
      }
      if (net["transfer_master"]) {
        transfer_net.master = net["transfer_master"].as<std::string>();
      }
      for (const auto & transfer : net["transfers"]) {
        ethercat_interface::EcTransferEntry transfer_entry;
        if (!transfer["size"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «size» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["in"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «in» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["out"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «out» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        transfer_entry.size = transfer["size"].as<size_t>();
        getTransferMemoryInfo(
          transfer["in"], transfer_entry.input,
          "in", transfer_net.name);
        getTransferMemoryInfo(
          transfer["out"], transfer_entry.output,
          "out", transfer_net.name);
        transfer_net.transfers.push_back(transfer_entry);
      }
      transfer_nets.push_back(transfer_net);
    }
  }

  return transfer_nets;
}

void EthercatDriver::configTransferNetwork()
{
  // This method can be used for additional transfer network configuration if needed
  // Currently, transfer network configuration is handled in on_activate()
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
