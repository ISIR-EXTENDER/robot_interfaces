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
  /**
   * @class FrankaCartesianVelocity
   * @brief A specialized robot component for controlling a Franka Emika robot using Cartesian
   * velocity commands.
   *
   * This class inherits from `GenericComponent` and implements the specific logic required to
   * handle and set Cartesian velocity commands for a Franka robot. It is designed to be part of a
   * modular robot control framework where different components manage different types of commands.
   */
  class FrankaCartesianVelocity : public GenericComponent
  {
  public:
    /**
     * @brief Construct a new FrankaCartesianVelocity object.
     *
     * Initializes the component, setting up its command interface names and preparing it
     * to receive Cartesian velocity commands.
     */
    explicit FrankaCartesianVelocity();

    bool setCommand(const CommandVariant &command) override;

    CartesianPosition getCurrentEndEffectorPose() const override;

  private:
    const std::array<std::string, 6> cartesian_vel_names{"vx", "vy", "vz", "wx", "wy", "wz"};
  }; // class FrankaCartesianVelocity
} // namespace robot_interfaces