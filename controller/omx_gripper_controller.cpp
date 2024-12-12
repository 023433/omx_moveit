
#include "omx_moveit/omx_gripper_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

namespace omx_moveit_gripper_controller
{

auto logger_ = rclcpp::get_logger("omx_controller_RobotGripperController");

RobotGripperController::RobotGripperController() : controller_interface::ControllerInterface() {}
controller_interface::CallbackReturn RobotGripperController::on_init(){
  RCLCPP_INFO(logger_, "on_init");

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotGripperController::command_interface_configuration() const {
  RCLCPP_INFO(logger_, "command_interface_configuration");
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  return conf;
}

controller_interface::InterfaceConfiguration RobotGripperController::state_interface_configuration() const {
  RCLCPP_INFO(logger_, "state_interface_configuration");
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  return conf;
}

controller_interface::CallbackReturn RobotGripperController::on_configure(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_configure");

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_activate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_activate");

  return CallbackReturn::SUCCESS;
}


controller_interface::return_type RobotGripperController::update(const rclcpp::Time & time, const rclcpp::Duration & period){
  (void)time;
  (void)period;
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotGripperController::on_deactivate(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_deactivate");
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_cleanup(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_cleanup");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_error(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_error");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotGripperController::on_shutdown(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(logger_, "on_shutdown");
  return CallbackReturn::SUCCESS;
}

}  // namespace omx_moveit_gripper_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omx_moveit_gripper_controller::RobotGripperController, controller_interface::ControllerInterface)
