#pragma once

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  /**
   * @class ExplorerJointPosition
   * @brief A specialized robot component for controlling explorer using the joint position
   * interface.
   *
   * This class inherits from `GenericComponent` and implements the specific logic required to
   * handle and set joint position commands for explorer.
   */
  class ExplorerJointPosition : public GenericComponent
  {
  public:
    /**
     * @brief Construct a new Explorer Joint Position object.
     *
     * Initializes the component, setting up its command interface names and preparing it
     * to receive joint position commands.
     */
    explicit ExplorerJointPosition();

    bool setCommand(const CommandVariant &command) override;

  private:
    const std::array<std::string, 6> joint_names{"joint_1", "joint_2", "joint_3",
                                                 "joint_4", "joint_5", "joint_6"};
  }; // class FrankaCartesianVelocity
} // namespace robot_interfaces