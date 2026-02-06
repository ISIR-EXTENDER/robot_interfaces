#include "robot_interfaces/explorer_velocity_component.hpp"

namespace robot_interfaces
{
  ExplorerCartesianVelocity::ExplorerCartesianVelocity()
      : GenericComponent("explorer_cartesian", 0, 6)
  {
    RCLCPP_INFO(rclcpp::get_logger("ExplorerCartesianVelocity"),
                "Robot Interface - ExplorerCartesianVelocity Interface in use.");

    // setup specific command name
    for (const auto &jname : joint_names)
    {
      state_names.emplace_back(jname + "/velocity");
      state_names.emplace_back(jname + "/position");
    }
  }

  bool ExplorerCartesianVelocity::setCommand(const CommandVariant &command)
  {

    if (const auto *jcommand = std::get_if<CartesianVelocity>(&command))
    {
      std::vector<double> values{jcommand->linear.x(),  jcommand->linear.y(),
                                 jcommand->linear.z(),  jcommand->angular.x(),
                                 jcommand->angular.y(), jcommand->angular.z()};
      return set_values(values);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ExplorerCartesianVelocity"),
                   "Received an unsupported command type.");
      return false;
    }
  }
} // namespace robot_interfaces