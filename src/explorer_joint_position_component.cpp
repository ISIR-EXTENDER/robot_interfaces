#include "robot_interfaces/explorer_joint_position_component.hpp"

namespace robot_interfaces
{
  ExplorerJointPosition::ExplorerJointPosition() : GenericComponent("explorer_joint_position", 0, 6)
  {
    RCLCPP_INFO(rclcpp::get_logger("ExplorerJointPosition"),
                "Robot Interface - ExplorerJointPosition Interface in use.");

    // setup specific franka command name
    for (const auto &jname : joint_names)
    {
      std::string explorer_command_name = jname + "/position";
      command_names.emplace_back(explorer_command_name);

      state_names.emplace_back(jname + "/position");
    }
  }

  bool ExplorerJointPosition::setCommand(const CommandVariant &command)
  {
    if (const auto *jcommand = std::get_if<JointCommand>(&command))
    {
      if (jcommand->command.size() != command_names.size())
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("ExplorerJointPosition"),
            "Number of command values sent does not match the number of command_interface.");
        return false;
      }
      return set_values(jcommand->command);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ExplorerJointPosition"),
                   "Received an unsupported command type.");
      return false;
    }
  }
} // namespace robot_interfaces