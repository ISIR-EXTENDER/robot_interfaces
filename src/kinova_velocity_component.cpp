#include "robot_interfaces/kinova_velocity_component.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robot_interfaces
{
  KinovaCartesianVelocity::KinovaCartesianVelocity()
      : GenericComponent("kinova_cartesian_velocity", /*state_interface_size*/ 0,
                         /*command_interface_size*/ 6)
  {
    RCLCPP_INFO(rclcpp::get_logger("KinovaCartesianVelocity"),
                "Kinova Gen3 (7-DOF) Cartesian Velocity Interface initialized.");

    // Set up command interface names
    command_names.clear();
    for (const auto &command_interface_name : command_interface_names_)
    {
      command_names.emplace_back(command_interface_name);
    }

    for (size_t i = 1; i <= 7; i++)
    {
      std::string franka_state_name = "joint_" + std::to_string(i) + "/position";
      state_names.emplace_back(franka_state_name);
      franka_state_name = "joint_" + std::to_string(i) + "/velocity";
      state_names.emplace_back(franka_state_name);
      franka_state_name = "joint_" + std::to_string(i) + "/effort";
      state_names.emplace_back(franka_state_name);
    }
  }

  KinovaCartesianVelocity::~KinovaCartesianVelocity() = default;

  bool KinovaCartesianVelocity::setCommand(const CommandVariant &command)
  {
    const auto *vel_command = std::get_if<CartesianVelocity>(&command);
    if (!vel_command)
    {
      RCLCPP_ERROR(rclcpp::get_logger("KinovaCartesianVelocity"),
                   "Received an unsupported command type (expected CartesianVelocity).");
      return false;
    }

    if (command_interfaces.size() != command_interface_names_.size())
    {
      RCLCPP_ERROR(rclcpp::get_logger("KinovaCartesianVelocity"),
                   "Mismatch between command interfaces (%zu) and expected size (%zu).",
                   command_interfaces.size(), command_interface_names_.size());
      return false;
    }

    std::vector<double> values{
        vel_command->linear.x(),  
        vel_command->linear.y(),  
        vel_command->linear.z(),  
        vel_command->angular.x(), 
        vel_command->angular.y(), 
        vel_command->angular.z() 
    };

    // Log at INFO level if any non-zero command
    /*
    if (std::any_of(values.begin(), values.end(), [](double v) { return std::abs(v) > 0.001; }))
    {
      RCLCPP_INFO(rclcpp::get_logger("KinovaCartesianVelocity"),
                   "setCommand: lin[%.4f, %.4f, %.4f] ang[%.4f, %.4f, %.4f]",
                   values[0], values[1], values[2], values[3], values[4], values[5]);
    }
    */

    bool success = set_values(values);
    if (!success)
    {
      RCLCPP_WARN(rclcpp::get_logger("KinovaCartesianVelocity"),
                  "Failed to set command values");
    }
    return success;
  }
} // namespace robot_interfaces
