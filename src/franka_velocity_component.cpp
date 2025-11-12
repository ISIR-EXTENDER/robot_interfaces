#include "robot_interfaces/franka_velocity_component.hpp"

namespace robot_interfaces
{
  FrankaCartesianVelocity::FrankaCartesianVelocity()
      : GenericComponent("cartesian_velocity_command", 0, 6)
  {
    RCLCPP_INFO(rclcpp::get_logger("FrankaCartesianVelocity"),
                "Robot Interface - Franka Cartesian Velocity Interface in use.");

    // setup specific franka command name
    for (const auto &vel_name : cartesian_vel_names)
    {
      std::string franka_command_name = vel_name + "/cartesian_velocity";
      command_names.push_back(franka_command_name);
    }
  }

  bool FrankaCartesianVelocity::setCommand(const CommandVariant &command)
  {
    if (const auto *vel_command = std::get_if<CartesianVelocityCommand>(&command))
    {
      std::vector<double> full_command{vel_command->linear.x(),  vel_command->linear.y(),
                                       vel_command->linear.z(),  vel_command->angular.x(),
                                       vel_command->angular.y(), vel_command->angular.z()};

      return set_values(full_command); // Or whatever your set_values function is called
    }
    else  
    {
      RCLCPP_ERROR(rclcpp::get_logger("FrankaCartesianVelocity"),
                   "Received an unsupported command type.");
      return false;
    }
  }

} // namespace robot_interfaces