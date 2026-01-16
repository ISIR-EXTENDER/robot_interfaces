#pragma once

#include <variant>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace robot_interfaces
{
  /// @brief Generic cartesian velocity for a manipulator end effector
  struct CartesianVelocity
  {
    /// @brief Linear velocity (x, y, z)
    Eigen::Vector3d linear;
    /// @brief Angular velocity (roll, pitch, yaw)
    Eigen::Vector3d angular;
  };
  /// @brief Generic cartesian position for a manipulator end effector
  struct CartesianPosition
  {
    /// @brief Target position (x, y, z)
    Eigen::Vector3d translation;
    /// @brief Target orientation as a quaternion
    Eigen::Quaterniond quaternion;
  };

  /// @brief Generic joint command. Can be used for position, velocity or effort.
  struct JointCommand
  {
    /// @brief Vector of command. Must be the same size as the command interface vector
    std::vector<double> command;
  };

  /// @brief A variant to hold all the possible commands
  using CommandVariant = std::variant<CartesianVelocity, CartesianPosition, JointCommand>;
} // namespace robot_interfaces