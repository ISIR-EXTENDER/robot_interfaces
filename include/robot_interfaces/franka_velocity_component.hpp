#pragma once

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "franka/control_types.h"
#include "franka/robot_state.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  class FrankaCartesianVelocity : public GenericComponent
  {
  public:
    explicit FrankaCartesianVelocity();

    bool setCommand(const CommandVariant &command) override;

  private:
    const std::array<std::string, 6> cartesian_vel_names{"vx", "vy", "vz", "wx", "wy", "wz"};

    const std::string cartesian_vel_command_name{"cartesian_velocity"};
  }; // class FrankaCartesianVelocity
} // namespace robot_interfaces