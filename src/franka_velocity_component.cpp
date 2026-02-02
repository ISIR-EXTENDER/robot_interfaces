#include "robot_interfaces/franka_velocity_component.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>

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
    // for (size_t i = 0; i < 16; i++)
    // {
    //   std::string franka_state_name = std::to_string(i) + "/cartesian_pose_state";
    //   state_names.emplace_back(franka_state_name);
    // }
    // joints
    for (size_t i = 1; i <= 7; i++)
    {
      std::string franka_state_name = "fr3_joint" + std::to_string(i) + "/position";
      state_names.emplace_back(franka_state_name);
      franka_state_name = "fr3_joint" + std::to_string(i) + "/velocity";
      state_names.emplace_back(franka_state_name);
      franka_state_name = "fr3_joint" + std::to_string(i) + "/effort";
      state_names.emplace_back(franka_state_name);
    }
  }

  void FrankaCartesianVelocity::set_commands_names(std::vector<std::string> custom_names)
  {
    if (command_names.empty())
    {
      for (size_t i = 0; i < command_names.size(); ++i)
      {
        command_names.emplace_back(component_name + "/" + std::to_string(i + 1));
      }
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


  CartesianPosition FrankaCartesianVelocity::getPoseFromStateInterfaces() const
  {
    constexpr size_t cartesian_pose_size = 16;
    const auto all_states = get_states_values();
    if (all_states.size() < cartesian_pose_size)
    {
      static bool warned_once = false;
      if (!warned_once)
      {
        RCLCPP_WARN(rclcpp::get_logger("FrankaCartesianVelocity"),
                    "State pose unavailable: expected %zu values (4x4 pose), got %zu. Returning "
                    "neutral pose.",
                    cartesian_pose_size, all_states.size());
        warned_once = true;
      }
      // Not enough data available; return a neutral pose to allow comparison callsites to proceed.
      return {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
    }

    std::array<double, cartesian_pose_size> current_pose_array{};
    std::copy_n(all_states.begin(), cartesian_pose_size, current_pose_array.begin());

    const Eigen::Matrix4d pose_matrix =
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(current_pose_array.data());

    Eigen::Quaterniond orientation(pose_matrix.block<3, 3>(0, 0));
    orientation.normalize();
    const Eigen::Vector3d translation(pose_matrix.block<3, 1>(0, 3));

    return {translation, orientation};
  }

  bool FrankaCartesianVelocity::hasStatePose() const
  {
    constexpr size_t cartesian_pose_size = 16;
    const auto all_states = get_states_values();
    return all_states.size() >= cartesian_pose_size;
  }

} // namespace robot_interfaces