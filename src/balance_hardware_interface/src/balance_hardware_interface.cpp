// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <limits>
#include <vector>

#include "balance_hardware_interface/balance_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace balance_hardware_interface
{
hardware_interface::CallbackReturn BalanceHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  hw_states_.assign(info_.joints.size(), 0);
  hw_commands_.assign(info_.joints.size(), 0);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BalanceHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_states_.assign(info_.joints.size(), 0);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BalanceHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BalanceHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn BalanceHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BalanceHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BalanceHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BalanceHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < hw_states_.size(); ++i) {
    hw_states_[i] = hw_commands_[i];
  }
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("BalanceHardwareInterface"),
      "Joint '%s' 新指令: %f",
      info_.joints[i].name.c_str(), hw_commands_[i]
    );
  }
  return hardware_interface::return_type::OK;
}

}  // namespace balance_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  balance_hardware_interface::BalanceHardwareInterface, hardware_interface::SystemInterface)
