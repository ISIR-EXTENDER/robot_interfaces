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
      state_names.emplace_back(jname + "/velocity");
      state_names.emplace_back(jname + "/effort");
    }
  }

  bool ExplorerJointPosition::setCommand(const CommandVariant &command)
  {
    if (const auto *vel_command = std::get_if<JointCommand>(&command))
    {
      return set_values(vel_command->command);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ExplorerJointPosition"),
                   "Received an unsupported command type.");
      return false;
    }
  }
  
  CartesianPosition ExplorerJointPosition::getCurrentEndEffectorPose() const
  {
    // TODO: Implement retrieval of the actual end-effector pose from state interfaces if available.
    CartesianPosition pose;
    pose.translation = Eigen::Vector3d::Zero();
    pose.quaternion = Eigen::Quaterniond::Identity();
    return pose;
  }

} // namespace robot_interfaces