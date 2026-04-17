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
     * @brief Construct a new Joint Position object.
     *
     * Initializes the component, setting up its command interface names and preparing it
     * to receive joint position commands.
     */
    explicit GenericJointPosition();
    GenericJointPosition(const std::vector<std::string> &jnames);

    Eigen::VectorXd getLowerPositionJointLimits() const;

    Eigen::VectorXd getUpperPositionJointLimits() const;

    Eigen::VectorXd getVelocityJointLimits() const;

    Eigen::VectorXd getEffortJointLimits() const;

    std::vector<std::string> getJointNames() const
    {

      return joint_names;
    }

    bool setCommand(const CommandVariant &command) override;

  private:
    /**
     * @brief Internal helper to extract limits for the specified joint_names.
     * @param source_vector The global Pinocchio limit vector (e.g., model.lowerPositionLimit)
     * @param use_v_idx If true, uses idx_v/nv (velocity/effort). If false, uses idx_q/nq
     * (position).
     */
    Eigen::VectorXd getLimitsInternal(const Eigen::VectorXd &source_vector, bool use_v_idx) const;
    std::vector<std::string> joint_names;
  }; // class GenericJointPosition
} // namespace robot_interfaces