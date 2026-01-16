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
      std::string explorer_command_name = jname + "/velocity";
      command_names.emplace_back(explorer_command_name);

      state_names.emplace_back(jname + "/velocity");
      state_names.emplace_back(jname + "/position");
    }
  }

  bool ExplorerCartesianVelocity::initKinematics(const std::string &urdf_xml,
                                                 const std::string &base_frame,
                                                 const std::string &tool_frame)
  {
    if (!GenericComponent::initKinematics(urdf_xml, base_frame, tool_frame))
    {
      return false;
    }

    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_);

    // Set damping to handle singularities
    ik_vel_solver_->setLambda(0.01);

    qdot_out_.resize(kdl_chain_.getNrOfJoints());

    RCLCPP_INFO(rclcpp::get_logger("ExplorerCartesianVelocity"),
                "WDLS Velocity Solver initialized.");

    return true;
  }

  bool ExplorerCartesianVelocity::setCommand(const CommandVariant &command)
  {

    if (const auto *jcommand = std::get_if<CartesianVelocity>(&command))
    {
      for (size_t i = 0; i < kdl_joint_to_interface_index_.size(); ++i)
      {
        size_t interface_idx = kdl_joint_to_interface_index_[i];
        kdl_joint_angles_(i) = state_interfaces[interface_idx].get().get_value();
      }

      KDL::Twist twist_in;
      twist_in.vel.x(jcommand->linear[0]);
      twist_in.vel.y(jcommand->linear[1]);
      twist_in.vel.z(jcommand->linear[2]);
      twist_in.rot.x(jcommand->angular[0]);
      twist_in.rot.y(jcommand->angular[1]);
      twist_in.rot.z(jcommand->angular[2]);

      // J# * Twist
      if (ik_vel_solver_->CartToJnt(kdl_joint_angles_, twist_in, qdot_out_) < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("ExplorerCartesianVelocity"), "WDLS Solver failed.");
        return false;
      }

      std::vector<double> joint_vel_cmds(qdot_out_.rows());
      for (unsigned int i = 0; i < qdot_out_.rows(); ++i)
      {
        joint_vel_cmds[i] = qdot_out_(i);
      }

      return set_values(joint_vel_cmds);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ExplorerCartesianVelocity"),
                   "Received an unsupported command type.");
      return false;
    }
  }
} // namespace robot_interfaces