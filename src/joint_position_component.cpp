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
      std::string explorer_command_name = jname + "/position";
      command_names.emplace_back(explorer_command_name);

      state_names.emplace_back(jname + "/position");
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
      std::string explorer_command_name = jname + "/position";
      command_names.emplace_back(explorer_command_name);

      state_names.emplace_back(jname + "/position");
    }
  }

  bool GenericJointPosition::initKinematics(const std::string &urdf_xml,
                                            const std::string &base_frame,
                                            const std::string &tool_frame)
  {
    if (!GenericComponent::initKinematics(urdf_xml, base_frame, tool_frame))
    {
      return false;
    }

    for (const std::string &jname : joint_names)
    {
      JointLimits jlimit;
      jlimit.has_position_limits = false;

      auto joint = urdf_model.getJoint(jname);
      if (joint)
      {
        switch (joint->type)
        {
        case urdf::Joint::REVOLUTE:
          jlimit.jtype = JointType::REVOLUTE;
          break;
        case urdf::Joint::CONTINUOUS:
          jlimit.jtype = JointType::CONTINUOUS;
          break;
        case urdf::Joint::PRISMATIC:
          jlimit.jtype = JointType::PRISMATIC;
          break;
        case urdf::Joint::FIXED:
          jlimit.jtype = JointType::FIXED;
          break;
        case urdf::Joint::FLOATING:
          jlimit.jtype = JointType::FLOATING;
          break;
        case urdf::Joint::PLANAR:
          jlimit.jtype = JointType::PLANAR;
          break;
        default:
          jlimit.jtype = JointType::UNKNOWN;
          break;
        }
      }

      if (joint->limits)
      {
        jlimit.min_position = joint->limits->lower;
        jlimit.max_position = joint->limits->upper;
        jlimit.has_position_limits = true;
        jlimit.max_velocity = joint->limits->velocity;
        jlimit.has_velocity_limits = true;
      }
      joint_limits_.push_back(jlimit);
    }
    return true;
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