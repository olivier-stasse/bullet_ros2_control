// Copyright 2022 LAAS-CNRS
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

#include <memory>
#include <string>
#include <vector>

#include "bullet_ros2_control/bullet_system.hpp"

class bullet_ros2_control::BulletSystemPrivate
{
 public:
  BulletSystemPrivate() = default;

  ~BulletSystemPrivate() = default;

  /// \brief Degrees of freedom.
  size_t n_dof_;

  /// \brief Shared pointer to the simulator
  std::shared_ptr<b3RobotSimulatorClientAPI_NoGUI> sim_;

    /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// Bullet robot unique ID.
  int robotUniqueID_;
  
  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  /// \brief vector with the control method defined in the URDF for each joint.
  std::vector<BulletSystemInterface::ControlMethod> joint_control_methods_;

  /// \brief handles to the joints from within Bullet
  std::map<std::string,int> joints_name_to_id_;

  /// \brief vector with the current joint position
  std::vector<double> joint_position_;

  /// \brief vector with the current joint velocity
  std::vector<double> joint_velocity_;

  /// \brief vector with the current joint effort
  std::vector<double> joint_effort_;

  /// \brief vector with the current cmd joint position
  std::vector<double> joint_position_cmd_;

  /// \brief vector with the current cmd joint velocity
  std::vector<double> joint_velocity_cmd_;

  /// \brief vector with the current cmd joint effort
  std::vector<double> joint_effort_cmd_;

  /// \brief The current positions of the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_pos_state_;

  /// \brief The current velocities of the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_vel_state_;

  /// \brief The current effort forces applied to the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_eff_state_;

  /// \brief The position command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_pos_cmd_;

  /// \brief The velocity command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_vel_cmd_;

  /// \brief The effort command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_eff_cmd_;
};

namespace bullet_ros2_control
{

bool BulletSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::shared_ptr<b3RobotSimulatorClientAPI_NoGUI> sim,
  const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO_STREAM(model_nh->get_logger(),"initSim - starting");
  dataPtr_ = std::make_unique<BulletSystemPrivate>();
  dataPtr_->last_update_sim_time_ros_ = rclcpp::Time();

  nh_ = model_nh;
  dataPtr_->sim_ = sim;
  dataPtr_->n_dof_ = hardware_info.joints.size();

  dataPtr_->joint_names_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_control_methods_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_position_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_velocity_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_effort_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_pos_state_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_vel_state_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_eff_state_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_position_cmd_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_velocity_cmd_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_effort_cmd_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_pos_cmd_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_vel_cmd_.resize(dataPtr_->n_dof_);
  dataPtr_->joint_eff_cmd_.resize(dataPtr_->n_dof_);

  // Checking if bullet is working.
  if (!dataPtr_->sim_->isConnected()) {
    RCLCPP_ERROR(nh_->get_logger(), "No physics engine configured in Bullet.");
    return false;
  }

  if (dataPtr_->n_dof_ == 0) {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "There is not joint available ");
    return false;
  }
  RCLCPP_INFO_STREAM(nh_->get_logger(),"initSim - initialize joints");

  for (unsigned int j = 0; j < dataPtr_->n_dof_; j++) {
    std::string joint_name = dataPtr_->joint_names_[j] = hardware_info.joints[j].name;

    b3JointInfo aJointInfo;
    bool bSimJoint = sim->getJointInfo(dataPtr_->robotUniqueID_,(int)j,&aJointInfo);
    if (!bSimJoint) {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the Bullet model.");
      continue;
    }
    dataPtr_->joints_name_to_id_[joint_name]=(int)j;

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Loading joint: " << joint_name);

    RCLCPP_INFO_STREAM(nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); i++) {
      if (hardware_info.joints[j].command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "\t\t position");
        dataPtr_->joint_control_methods_[j] |= POSITION;
        dataPtr_->joint_pos_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_POSITION, &dataPtr_->joint_position_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "\t\t velocity");
        dataPtr_->joint_control_methods_[j] |= VELOCITY;
        dataPtr_->joint_vel_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_VELOCITY, &dataPtr_->joint_velocity_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name == "effort") {
        dataPtr_->joint_control_methods_[j] |= EFFORT;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "\t\t effort");
        dataPtr_->joint_eff_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_EFFORT, &dataPtr_->joint_effort_cmd_[j]);
      }
    }

    RCLCPP_INFO_STREAM( nh_->get_logger(), "\tState:");
    // register the state handles
    for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); i++) {
      if (hardware_info.joints[j].state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "\t\t position");
        dataPtr_->joint_pos_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_POSITION, &dataPtr_->joint_position_cmd_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "\t\t velocity");
        dataPtr_->joint_vel_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_VELOCITY, &dataPtr_->joint_velocity_cmd_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "\t\t effort");
        dataPtr_->joint_eff_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_EFFORT, &dataPtr_->joint_effort_cmd_[j]);
      }
    }
  }

  RCLCPP_INFO_STREAM(nh_->get_logger(), "Finished initSim");
  return true;
}

hardware_interface::CallbackReturn
BulletSystem::on_init(const hardware_interface::HardwareInfo & /* actuator_info */)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BulletSystem::export_state_interfaces()
{
  RCLCPP_INFO(nh_->get_logger(), "initSim - export_state_interfaces start");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (unsigned int i = 0; i < dataPtr_->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        dataPtr_->joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &dataPtr_->joint_position_[i]));
  }
  for (unsigned int i = 0; i < dataPtr_->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        dataPtr_->joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &dataPtr_->joint_velocity_[i]));
  }
  for (unsigned int i = 0; i < dataPtr_->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        dataPtr_->joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &dataPtr_->joint_effort_[i]));
  }
  RCLCPP_INFO(nh_->get_logger(), "initSim - export_state_interfaces end");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BulletSystem::export_command_interfaces()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "initSim - export_command_interfaces start");  
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int i = 0; i < dataPtr_->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        dataPtr_->joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &dataPtr_->joint_position_cmd_[i]));
  }
  for (unsigned int i = 0; i < dataPtr_->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        dataPtr_->joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &dataPtr_->joint_velocity_cmd_[i]));
  }
  for (unsigned int i = 0; i < dataPtr_->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        dataPtr_->joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &dataPtr_->joint_effort_cmd_[i]));
  }
  RCLCPP_INFO_STREAM(nh_->get_logger(), "initSim - export_command_interfaces end");
  return command_interfaces;
}


hardware_interface::return_type BulletSystem::read(const rclcpp::Time & /* time */,
                                                   const rclcpp::Duration & /* period */)
{
  
  for (unsigned int j = 0; j < dataPtr_->joint_names_.size(); j++) {
    b3JointSensorState state;
    dataPtr_->sim_->getJointState(dataPtr_->robotUniqueID_, j, &state);
                  
    dataPtr_->joint_position_[j] = state.m_jointPosition;
    dataPtr_->joint_velocity_[j] = state.m_jointVelocity;
    dataPtr_->joint_effort_[j] = state.m_jointMotorTorque;
    
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BulletSystem::write(const rclcpp::Time & /* time */,
                                                    const rclcpp::Duration & /* period */)
{
  // Get the simulation time and period
  rclcpp::Time current_time_now = rclcpp::Time();
  rclcpp::Time sim_time_ros(current_time_now.seconds(),
                            current_time_now.nanoseconds());
  rclcpp::Duration sim_period = sim_time_ros - dataPtr_->last_update_sim_time_ros_;

  for (unsigned int j = 0; j < dataPtr_->joint_names_.size(); j++) {
    if (dataPtr_->joint_control_methods_[j] & POSITION) {
      b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
      controlArgs.m_targetPosition = dataPtr_->joint_position_cmd_[j];
      dataPtr_->sim_->setJointMotorControl(
          dataPtr_->robotUniqueID_,
          dataPtr_->joints_name_to_id_[dataPtr_->joint_names_[j]],
          controlArgs);
    }
    if (dataPtr_->joint_control_methods_[j] & VELOCITY) {
      b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
      controlArgs.m_targetPosition = dataPtr_->joint_position_cmd_[j];
      controlArgs.m_targetPosition = dataPtr_->joint_velocity_cmd_[j];
      dataPtr_->sim_->setJointMotorControl(
          dataPtr_->robotUniqueID_,
          dataPtr_->joints_name_to_id_[dataPtr_->joint_names_[j]],
          controlArgs);
    }
    if (dataPtr_->joint_control_methods_[j] & EFFORT) {
      b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
      controlArgs.m_maxTorqueValue = dataPtr_->joint_effort_cmd_[j];
      dataPtr_->sim_->setJointMotorControl(
          dataPtr_->robotUniqueID_,
          dataPtr_->joints_name_to_id_[dataPtr_->joint_names_[j]],
          controlArgs);
    }
  }

  dataPtr_->last_update_sim_time_ros_ = sim_time_ros;

  return hardware_interface::return_type::OK;
}
}  // namespace bullet_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  bullet_ros2_control::BulletSystem, bullet_ros2_control::BulletSystemInterface)

  
