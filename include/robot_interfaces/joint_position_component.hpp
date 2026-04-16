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
   * @brief A specialized robot component for controlling a robot using the joint position
   * interface.
   *
   * This class inherits from `GenericComponent` and implements the specific logic required to
   * handle and set joint position commands for explorer.
   */
  class GenericJointPosition : public GenericComponent
  {
  public:
    /**
     * @brief Construct a new Explorer Joint Position object.
     *
     * Initializes the component, setting up its command interface names and preparing it
     * to receive joint position commands.
     */
    explicit GenericJointPosition();
    GenericJointPosition(const std::vector<std::string> &jnames);

    Eigen::VectorXd getLowerJointLimits() const
    {
      return model.lowerPositionLimit;
    }
    Eigen::VectorXd getUpperJointLimits() const
    {
      return model.upperPositionLimit;
    }

    Eigen::VectorXd getVelocityJointLimits() const
    {
      return model.velocityLimit;
    }

    Eigen::VectorXd getEffortJointLimits() const
    {
      return model.effortLimit;
    }

    std::vector<std::string> getJointNames() const
    {
      return std::vector<std::string>(joint_names.begin(), joint_names.end());
    }

    bool setCommand(const CommandVariant &command) override;

  private:
    std::vector<std::string> joint_names;
  }; // class GenericJointPosition
} // namespace robot_interfaces