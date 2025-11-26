#include "robot_interfaces/kinova_velocity_component.hpp"

namespace robot_interfaces
{
  KinovaCartesianVelocity::KinovaCartesianVelocity()
      : GenericComponent("kinova_cartesian_velocity", /*state_interface_size*/ 0,
                         /*command_interface_size*/ 6)
  {
    RCLCPP_INFO(rclcpp::get_logger("KinovaCartesianVelocity"),
                "Robot Interface - Kinova Gen3 Cartesian Velocity Interface in use.");

    command_names.clear();
    // Use specific Kinova hardware interface names for velocity commands:
    for (const auto &name : command_interface_names_)
    {
      command_names.emplace_back(name);
    }

    // TODO: No state interfaces for now.
    state_names.clear();
  }

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

    return set_values(values);
  }

  CartesianPosition KinovaCartesianVelocity::getCurrentEndEffectorPose() const
  {
    // TODO: Implement retrieval of the actual end-effector pose from state interfaces if available.
    CartesianPosition pose;
    pose.translation = Eigen::Vector3d::Zero();
    pose.quaternion = Eigen::Quaterniond::Identity();
    return pose;
  }

} // namespace robot_interfaces
