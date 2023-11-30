// Copyright 2022 LAAS/CNRS
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

#ifndef BULLET_ROS2_CONTROL__BULLET_SYSTEM_HPP_
#define BULLET_ROS2_CONTROL__BULLET_SYSTEM_HPP_

#include "bullet_ros2_control/bullet_system_interface.hpp"

namespace bullet_ros2_control
{
// Forward declaration
class BulletSystemPrivate;

// These class must inherit `bullet_ros2_control::GazeboSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

class BulletSystem : public BulletSystemInterface
{
public:
  // Documentation Inherited
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Documentation Inherited
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::shared_ptr<b3RobotSimulatorClientAPI_NoGUI> sim,
    const hardware_interface::HardwareInfo & hardware_info) override;

private:
  /// \brief Private data class
  std::unique_ptr<BulletSystemPrivate> dataPtr_;
};

}  // namespace gazebo_ros2_control

#endif /* PYBULLET_ROS2_CONTROL__BULLET_SYSTEM_HPP_ */
