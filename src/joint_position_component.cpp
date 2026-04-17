#include "robot_interfaces/joint_position_component.hpp"

namespace robot_interfaces
{
  GenericJointPosition::GenericJointPosition() : GenericComponent("generic_joint_position", 0, 6)
  {
    RCLCPP_INFO(rclcpp::get_logger("GenericJointPosition"),
                "Robot Interface - GenericJointPosition Interface in use.");

    // setup specific command name
    for (const auto &jname : joint_names)
    {
      std::string command_name_tmp = jname + "/position";
      command_names.emplace_back(command_name_tmp);

      state_names.emplace_back(jname + "/position");
      state_names.emplace_back(jname + "/velocity");
      state_names.emplace_back(jname + "/effort");
    }
  }

  GenericJointPosition::GenericJointPosition(const std::vector<std::string> &jnames)
      : GenericComponent("generic_joint_position", 0, 6), joint_names(jnames)
  {
    RCLCPP_INFO(rclcpp::get_logger("GenericJointPosition"),
                "Robot Interface - GenericJointPosition Interface in use.");

    // setup specific command name
    for (const auto &jname : joint_names)
    {
      std::string command_name_tmp = jname + "/position";
      command_names.emplace_back(command_name_tmp);

      state_names.emplace_back(jname + "/position");
      state_names.emplace_back(jname + "/velocity");
      state_names.emplace_back(jname + "/effort");
    }
  }

  Eigen::VectorXd GenericJointPosition::getLowerPositionJointLimits() const
  {
    return getLimitsInternal(model.lowerPositionLimit, false);
  }

  Eigen::VectorXd GenericJointPosition::getUpperPositionJointLimits() const
  {
    return getLimitsInternal(model.upperPositionLimit, false);
  }

  Eigen::VectorXd GenericJointPosition::getVelocityJointLimits() const
  {
    return getLimitsInternal(model.velocityLimit, true);
  }

  Eigen::VectorXd GenericJointPosition::getEffortJointLimits() const
  {
    return getLimitsInternal(model.effortLimit, true);
  }

  Eigen::VectorXd GenericJointPosition::getLimitsInternal(const Eigen::VectorXd &source_vector,
                                                          bool use_v_idx) const
  {
    int total_dim = 0;
    for (const auto &name : joint_names)
    {
      if (model.existJointName(name))
      {
        auto id = model.getJointId(name);
        total_dim += (use_v_idx ? model.joints[id].nv() : model.joints[id].nq());
      }
    }

    Eigen::VectorXd result(total_dim);
    int current_offset = 0;

    for (const auto &name : joint_names)
    {
      if (!model.existJointName(name))
        continue;

      auto id = model.getJointId(name);
      int start_idx = (use_v_idx ? model.joints[id].idx_v() : model.joints[id].idx_q());
      int size = (use_v_idx ? model.joints[id].nv() : model.joints[id].nq());

      result.segment(current_offset, size) = source_vector.segment(start_idx, size);

      current_offset += size;
    }

    return result;
  }

  bool GenericJointPosition::setCommand(const CommandVariant &command)
  {
    if (const auto *jcommand = std::get_if<JointCommand>(&command))
    {
      if (jcommand->command.size() != command_names.size())
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("GenericJointPosition"),
            "Number of command values sent does not match the number of command_interface.");
        return false;
      }
      return set_values(jcommand->command);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("GenericJointPosition"),
                   "Received an unsupported command type.");
      return false;
    }
  }
} // namespace robot_interfaces