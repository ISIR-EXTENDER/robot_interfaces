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
      command_names.emplace_back(franka_command_name);
    }
    // setup states for franka
    // cartesian pose of end effector
    for (size_t i = 0; i < 16; i++)
    {
      std::string franka_state_name = std::to_string(i) + "/cartesian_pose_state";
      state_names.emplace_back(franka_state_name);
    }
    // joints
    for (size_t i = 0; i < 7; i++)
    {
      std::string franka_state_name = "fr3_joint" + std::to_string(i) + "/position";
      state_names.emplace_back(franka_state_name);
      franka_state_name = "fr3_joint" + std::to_string(i) + "/velocity";
      state_names.emplace_back(franka_state_name);
      franka_state_name = "fr3_joint" + std::to_string(i) + "/effort";
      state_names.emplace_back(franka_state_name);
    }
  }

  bool FrankaCartesianVelocity::setCommand(const CommandVariant &command)
  {
    if (const auto *vel_command = std::get_if<CartesianVelocity>(&command))
    {
      std::vector<double> full_command{vel_command->linear.x(),  vel_command->linear.y(),
                                       vel_command->linear.z(),  vel_command->angular.x(),
                                       vel_command->angular.y(), vel_command->angular.z()};

      return set_values(full_command); 
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("FrankaCartesianVelocity"),
                   "Received an unsupported command type.");
      return false;
    }
  }

  CartesianPosition FrankaCartesianVelocity::getCurrentEndEffectorPose() const
  {
    constexpr size_t cartesian_pose_size = 16;
    std::array<double, cartesian_pose_size> current_pose_array{};

    const auto all_states = get_states_values();

    std::copy_n(all_states.begin(), cartesian_pose_size, current_pose_array.begin());

    Eigen::Matrix4d pose_matrix =
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(current_pose_array.data());

    Eigen::Quaterniond orientation(pose_matrix.block<3, 3>(0, 0));
    orientation.normalize();
    Eigen::Vector3d translation(pose_matrix.block<3, 1>(0, 3));
    
    return {translation, orientation};
  }

} // namespace robot_interfaces