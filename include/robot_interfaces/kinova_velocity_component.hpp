#pragma once

#include <array>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  /**
   * @class KinovaCartesianVelocity
   * @brief A specialized robot component for controlling a Kinova Gen3 robot using Cartesian
   * velocity commands.
   *
   * This class inherits from `GenericComponent` and implements the specific logic required to
   * handle and set Cartesian velocity commands for a Kinova Gen3 robot. It is designed to be part
   * of a modular robot control framework where different components manage different types of
   * commands.
   */
  class KinovaCartesianVelocity : public GenericComponent
  {
  public:
    /**
     * @brief Construct a new KinovaCartesianVelocity object.
     *
     * Initializes the component, setting up its command interface names and preparing it
     * to receive Cartesian velocity commands.
     */
    explicit KinovaCartesianVelocity();

    bool setCommand(const CommandVariant &command) override;

    CartesianPosition getCurrentEndEffectorPose() const override;

  private:
    // Robot-specific hardware interface names for cartesian twist commands.
    // Order must match the order used in setCommand().
    const std::array<std::string, 6> command_interface_names_{
        "tcp/twist.linear.x",  "tcp/twist.linear.y",  "tcp/twist.linear.z",
        "tcp/twist.angular.x", "tcp/twist.angular.y", "tcp/twist.angular.z"};
  };
} // namespace robot_interfaces
