#pragma once

#include <Eigen/Dense>
#include <vector>

namespace robot_interfaces
{
/// @brief Interface for a controller that accepts a velocity command.
  class VelocityCommand
  {
  public:
    virtual ~VelocityCommand() = default;
    virtual bool setCommand(const Eigen::Vector3d &linear_vel, const Eigen::Vector3d &angular_vel) = 0;
  };

  /// @brief Interface for a controller that accepts a joint position command.
  class PositionCommand
  {
  public:
    virtual ~PositionCommand() = default;
    /// TBD
    virtual bool setCommand() = 0;
  };

  /// @brief Interface for a controller that accepts a joint torque command.
  class TorqueCommand
  {
  public:
    virtual ~TorqueCommand() = default;
    /// @brief TBD 
    virtual bool setCommand() = 0;
  };
}