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

#ifndef MFJA_YASKAWA_GZ_HARDWARE__YASKAWA_GAZEBO_SYSTEM_HPP_
#define MFJA_YASKAWA_GZ_HARDWARE__YASKAWA_GAZEBO_SYSTEM_HPP_

#include <map>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace mfja_yaskawa_gz_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Gazebo hardware system of the Yaskawa arms, with the interfaces of the real arms.
/**
 * Loaded by the gz_ros2_control Gazebo plugin in place of gz_ros2_control/GazeboSimSystem
 * ("custom gz_ros2_control simulation plugins" in the gz_ros2_control documentation).
 *
 * Joints: exactly the interfaces of motoros2_hw_interfaces/MotoROS2HardwareInterface, checked in
 * on_init(): commands position and velocity, states position and velocity. The velocity applied to
 * the simulated joint is computed by velocity_command(): feedforward of the commanded velocity plus
 * correction of the position error, as the robot controller follows the points queued in MotoROS2.
 * Joints whose commands are not claimed by any controller are held still.
 *
 * GPIOs (the gripper): every command is copied to the state of the same name, as
 * mock_components/GenericSystem does. The gripper itself is not simulated.
 *
 * Hardware parameter: position_gain (1/s, default 10).
 */
class YaskawaGazeboSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & ecm,
    unsigned int update_rate) override;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    sim::Entity entity{sim::kNullEntity};
    std::string position_interface;  // "<joint>/position"
    std::string velocity_interface;  // "<joint>/velocity"
    double position{0.0};
    double velocity{0.0};
    bool position_claimed{false};
    bool velocity_claimed{false};
  };

  std::vector<Joint> joints_;
  std::vector<std::string> gpio_interfaces_;  // "<gpio>/<interface>", command and state
  sim::EntityComponentManager * ecm_{nullptr};
  double position_gain_{10.0};
};

}  // namespace mfja_yaskawa_gz_hardware

#endif  // MFJA_YASKAWA_GZ_HARDWARE__YASKAWA_GAZEBO_SYSTEM_HPP_
