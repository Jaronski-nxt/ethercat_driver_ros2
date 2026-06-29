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

#ifndef ETHERCAT_INTERFACE__EC_SLAVE_HPP_
#define ETHERCAT_INTERFACE__EC_SLAVE_HPP_

#include <ecrt.h>
#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <string>

#include "ethercat_interface/ec_sdo_manager.hpp"

namespace ethercat_interface
{

class EcSlave
{
public:
  EcSlave(uint32_t vendor_id, uint32_t product_id)
  : vendor_id_(vendor_id),
    product_id_(product_id) {}
  EcSlave(uint32_t vendor_id, uint32_t product_id, uint16_t alias, uint16_t position)
  : vendor_id_(vendor_id),
    product_id_(product_id)
  {
    setAliasAndPosition(alias, position);
  }
  virtual ~EcSlave() {}

public:
  /** read or write data to the domain from the index of the entry in the recorded pdos */
  virtual void processData(size_t /*entry_idx*/, uint8_t * /*domain_address*/) {}
  /** a pointer to syncs. return &syncs[0] */
  virtual const ec_sync_info_t * syncs() {return NULL;}
  virtual bool initialized() {return true;}
  virtual void set_state_is_operational(bool value) {is_operational_ = value;}
  /** Assign activate DC synchronization. return activate word*/
  virtual int assign_activate_dc_sync() {return 0x00;}
  /** number of elements in the syncs array. */
  virtual size_t syncSize() {return 0;}
  /** a pointer to all PDO entries */
  virtual const ec_pdo_entry_info_t * channels() {return NULL;}
  /** whether the master should push PDO assignment/mapping config for this slave.
   *  Some slaves expose fixed PDO layout (CoE PDO assign/config disabled) and
   *  reject remapping; they should return false here while still using PDO I/O. */
  virtual bool configurePdos() {return true;}
  /** a map from domain index to pdo indices in that domain.
  *  map<domain index, vector<channels_ indices> > */
  typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;
  virtual void domains(DomainMap & /*domains*/) const {}
  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_parameters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    state_interface_ptr_ = state_interface;
    command_interface_ptr_ = command_interface;
    parameters_ = slave_parameters;
    return true;
  }

  // --- Readiness-gate extensions (appended at end to preserve vtable ABI) ---
  /** Returns true if the slave is fully ready to accept motion commands.
   *  For CiA402 drives this means the drive reached OPERATION_ENABLED.
   *  Default: equivalent to initialized() (bus-level readiness is enough for
   *  non-drive slaves such as grippers, FT-sensors or transfer modules). */
  virtual bool readyForCommands() {return initialized();}
  /** Returns true if the slave reports a valid (finite) actual position.
   *  Default: true (slaves without a position do not gate the startup). */
  virtual bool hasValidPosition() {return true;}
  /** Short human-readable readiness/status string for diagnostics/logging. */
  virtual std::string statusString() {return is_operational_ ? "operational" : "not-operational";}

  // --- Deterministic group-barrier startup extensions ---
  // (appended at the end of the class to preserve vtable/ABI compatibility)
  /** Returns the current CiA402 device state as an integer (DeviceState enum
   *  value) for drives, or -1 for slaves that are not CiA402 drives.
   *  The driver uses this to coordinate a synchronous, group-wide power-up
   *  ("barrier"): all drives must reach a given state before any of them is
   *  commanded to advance to the next one. */
  virtual int cia402State() {return -1;}
  /** Returns the power-up rank of the drive's current CiA402 state on the
   *  normal SWITCH_ON_DISABLED → READY_TO_SWITCH_ON → SWITCH_ON →
   *  OPERATION_ENABLED path (0, 1, 2, 3 respectively).
   *  Returns -1 for non-drive slaves, faults, or undefined states. */
  virtual int cia402PowerupRank() const {return -1;}
  /** Enable/disable the startup barrier for this slave and set the highest
   *  CiA402 state the slave is currently allowed to advance to.
   *  While the barrier is enabled, a drive that already reached (or passed) the
   *  target state must HOLD its control word instead of advancing further, so
   *  the whole group steps through the state machine together.
   *  Default: no-op (non-drive slaves ignore the barrier). */
  virtual void setStartupBarrier(bool /*enabled*/, int /*target_state*/) {}

public:
  inline
  void setAliasAndPosition(uint16_t alias, uint16_t position)
  {
    alias_ = alias;
    position_ = position;
    is_alias_and_position_set_ = true;
  }

  inline
  bool isAliasAndPositionSet()
  {
    return is_alias_and_position_set_;
  }

public:
  uint16_t alias_;        //< Slave alias.
  uint16_t position_;     //< Index after alias. If alias is zero, stores the ring position.
  uint32_t vendor_id_;   //< Slave vendor ID.
  uint32_t product_id_;  //< Slave product code.

  std::vector<SdoConfigEntry> sdo_config;

protected:
  std::vector<double> * state_interface_ptr_;
  std::vector<double> * command_interface_ptr_;
  std::unordered_map<std::string, std::string> parameters_;
  bool is_operational_ = false;
  bool is_alias_and_position_set_ = false;
};
}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SLAVE_HPP_
