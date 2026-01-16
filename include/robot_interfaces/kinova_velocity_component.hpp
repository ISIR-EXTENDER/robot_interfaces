#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  /**
   * @class KinovaCartesianVelocity
   * @brief Cartesian velocity controller for Kinova Gen3 7-DOF robot.
   *
   * Inherits KDL-based forward kinematics from GenericComponent.
   * Provides Cartesian twist commands via tcp/twist.* interfaces.
   */
  class KinovaCartesianVelocity : public GenericComponent
  {
  public:
    explicit KinovaCartesianVelocity();

    ~KinovaCartesianVelocity();

    bool setCommand(const CommandVariant &command) override;
    
  private:
    // Robot-specific hardware interface names for cartesian twist commands
    const std::array<std::string, 6> command_interface_names_{
        "tcp/twist.linear.x",  "tcp/twist.linear.y",  "tcp/twist.linear.z",
        "tcp/twist.angular.x", "tcp/twist.angular.y", "tcp/twist.angular.z"};
  };
} // namespace robot_interfaces
