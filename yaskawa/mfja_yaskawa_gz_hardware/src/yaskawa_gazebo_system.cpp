// Copyright 2026 Niel MBEDE
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

#include "mfja_yaskawa_gz_hardware/yaskawa_gazebo_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "mfja_yaskawa_gz_hardware/velocity_command.hpp"

namespace mfja_yaskawa_gz_hardware
{

namespace
{

// True when the interfaces are exactly the expected ones, in this order.
bool interfaces_are(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::vector<std::string> & expected)
{
  if (interfaces.size() != expected.size()) {
    return false;
  }
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (interfaces[i].name != expected[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool YaskawaGazeboSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & ecm,
  unsigned int /*update_rate*/)
{
  nh_ = model_nh;
  ecm_ = &ecm;
  const rclcpp::Logger logger = nh_->get_logger().get_child(hardware_info.name);

  const auto gain = hardware_info.hardware_parameters.find("position_gain");
  if (gain != hardware_info.hardware_parameters.end()) {
    try {
      position_gain_ = hardware_interface::stod(gain->second);
    } catch (const std::invalid_argument &) {
      RCLCPP_ERROR(logger, "position_gain '%s' is not a number.", gain->second.c_str());
      return false;
    }
  }

  for (const auto & joint_info : hardware_info.joints) {
    const auto entity = joints.find(joint_info.name);
    if (entity == joints.end()) {
      RCLCPP_ERROR(logger, "Joint '%s' is not in the Gazebo model.", joint_info.name.c_str());
      return false;
    }

    Joint joint;
    joint.name = joint_info.name;
    joint.entity = entity->second;
    joint.position_interface = joint_info.name + "/" + hardware_interface::HW_IF_POSITION;
    joint.velocity_interface = joint_info.name + "/" + hardware_interface::HW_IF_VELOCITY;

    // Gazebo fills these components only if they exist.
    if (!ecm.EntityHasComponentType(joint.entity, sim::components::JointPosition().TypeId())) {
      ecm.CreateComponent(joint.entity, sim::components::JointPosition());
    }
    if (!ecm.EntityHasComponentType(joint.entity, sim::components::JointVelocity().TypeId())) {
      ecm.CreateComponent(joint.entity, sim::components::JointVelocity());
    }

    // Initial position given in the description (initial_value of the position state).
    for (const auto & state : joint_info.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION && !state.initial_value.empty()) {
        try {
          joint.position = hardware_interface::stod(state.initial_value);
        } catch (const std::invalid_argument &) {
          RCLCPP_ERROR(
            logger, "Initial position '%s' of joint '%s' is not a number.",
            state.initial_value.c_str(), joint.name.c_str());
          return false;
        }
        ecm.CreateComponent(
          joint.entity, sim::components::JointPositionReset({joint.position}));
      }
    }
    joints_.push_back(joint);
  }

  for (const auto & gpio : hardware_info.gpios) {
    for (const auto & command : gpio.command_interfaces) {
      gpio_interfaces_.push_back(gpio.name + "/" + command.name);
    }
  }

  RCLCPP_INFO(
    logger, "%zu joints (position and velocity commands, position gain %.1f 1/s), "
    "%zu GPIO interfaces.", joints_.size(), position_gain_, gpio_interfaces_.size());
  return true;
}

CallbackReturn YaskawaGazeboSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Same joint interfaces as motoros2_hw_interfaces/MotoROS2HardwareInterface on the real arms.
  const std::vector<std::string> expected = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
  for (const auto & joint : info_.joints) {
    if (!interfaces_are(joint.command_interfaces, expected) ||
      !interfaces_are(joint.state_interfaces, expected))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' must have the command interfaces [position, velocity] and the state "
        "interfaces [position, velocity], as on the real arms.", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Every GPIO command is copied to the state of the same name.
  for (const auto & gpio : info_.gpios) {
    for (const auto & command : gpio.command_interfaces) {
      const bool has_state = std::any_of(
        gpio.state_interfaces.begin(), gpio.state_interfaces.end(),
        [&command](const hardware_interface::InterfaceInfo & state) {
          return state.name == command.name;
        });
      if (!has_state) {
        RCLCPP_ERROR(
          get_logger(), "GPIO '%s': command interface '%s' has no state interface of that name.",
          gpio.name.c_str(), command.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type YaskawaGazeboSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (auto & joint : joints_) {
    for (const auto & name : stop_interfaces) {
      if (name == joint.position_interface) {
        joint.position_claimed = false;
      } else if (name == joint.velocity_interface) {
        joint.velocity_claimed = false;
      }
    }
    for (const auto & name : start_interfaces) {
      if (name == joint.position_interface) {
        joint.position_claimed = true;
      } else if (name == joint.velocity_interface) {
        joint.velocity_claimed = true;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type YaskawaGazeboSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & joint : joints_) {
    const auto * position = ecm_->Component<sim::components::JointPosition>(joint.entity);
    const auto * velocity = ecm_->Component<sim::components::JointVelocity>(joint.entity);
    if (position != nullptr && !position->Data().empty()) {
      joint.position = position->Data()[0];
    }
    if (velocity != nullptr && !velocity->Data().empty()) {
      joint.velocity = velocity->Data()[0];
    }
    set_state(joint.position_interface, joint.position);
    set_state(joint.velocity_interface, joint.velocity);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type YaskawaGazeboSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & joint : joints_) {
    const double velocity = velocity_command(
      joint.position, joint.position_claimed, get_command<double>(joint.position_interface),
      joint.velocity_claimed, get_command<double>(joint.velocity_interface), position_gain_);

    auto * command = ecm_->Component<sim::components::JointVelocityCmd>(joint.entity);
    if (command == nullptr) {
      ecm_->CreateComponent(joint.entity, sim::components::JointVelocityCmd({velocity}));
    } else if (command->Data().empty()) {
      *command = sim::components::JointVelocityCmd({velocity});
    } else {
      command->Data()[0] = velocity;
    }
  }

  for (const auto & name : gpio_interfaces_) {
    const double value = get_command<double>(name);
    if (std::isfinite(value)) {
      set_state(name, value);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mfja_yaskawa_gz_hardware

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  mfja_yaskawa_gz_hardware::YaskawaGazeboSystem, gz_ros2_control::GazeboSimSystemInterface)
