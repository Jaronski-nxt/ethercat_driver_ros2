// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "ethercat_generic_plugins/cia402_common_defs.hpp"

namespace ethercat_generic_plugins
{

class EcCiA402Drive : public GenericEcSlave
{
public:
  EcCiA402Drive();
  virtual ~EcCiA402Drive();
  /** Returns true if drive has reached "operation enabled" state.
   *  The transition through the state machine is handled automatically. */
  bool initialized();

  /** True once the CiA402 device state machine reached OPERATION_ENABLED. */
  bool readyForCommands() override;
  /** True once a finite actual position (0x6064) has been received via TPDO. */
  bool hasValidPosition() override;
  /** CiA402 device-state name plus actual-position validity, for diagnostics. */
  std::string statusString() override;

  virtual void processData(size_t entry_idx, uint8_t * domain_address);

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_parameters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  int8_t mode_of_operation_display_ = 0;
  int8_t mode_of_operation_ = -1;

  void updateState();

  // --- Deterministic group-barrier startup ---
  /** Current CiA402 device state as int (DeviceState), for group coordination. */
  int cia402State() override {return static_cast<int>(state_);}
  /** Power-up rank of the current state (0=SWITCH_ON_DISABLED … 3=OPERATION_ENABLED),
   *  -1 for fault/quick-stop/undefined. Used by driver for group barrier coordination. */
  int cia402PowerupRank() const override {return powerupRankOf(state_);}
  /** Enable/disable barrier and set the highest CiA402 state this drive may
   *  advance to during the coordinated power-up. */
  void setStartupBarrier(bool enabled, int target_state) override
  {
    barrier_enabled_ = enabled;
    barrier_target_state_ = target_state;
  }

protected:
  uint32_t counter_ = 0;
  uint16_t last_status_word_ = -1;
  uint16_t status_word_ = 0;
  uint16_t control_word_ = 0;
  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;
  bool initialized_ = false;
  bool auto_fault_reset_ = false;
  bool auto_state_transitions_ = true;
  bool fault_reset_ = false;

  // --- Deterministic group-barrier startup ---
  /** When true, the drive must not advance past barrier_target_state_ on the
   *  normal power-up path (faults are always handled, so a fault reset still
   *  works). Controlled centrally by the EthercatDriver during on_activate(). */
  bool barrier_enabled_ = false;
  /** Highest CiA402 DeviceState the drive may advance to while the barrier is
   *  enabled. Default STATE_OPERATION_ENABLED = no restriction. */
  int barrier_target_state_ = static_cast<int>(STATE_OPERATION_ENABLED);
  int fault_reset_command_interface_index_ = -1;
  bool last_fault_reset_command_ = false;
  double last_position_ = std::numeric_limits<double>::quiet_NaN();

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word);
  /** power-up path rank for a given state (-1 if not on the forward power-up path). */
  static int powerupRankOf(DeviceState state);
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word);
  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node drive_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);

  bool position_hold_initialized_{false};
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
