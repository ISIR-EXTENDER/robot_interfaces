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
  /// @brief Enumeration of joint types that are available in urdf format
  enum class JointType
  {
    UNKNOWN,
    REVOLUTE,   // Rotates with limits (hinge)
    CONTINUOUS, // Rotates without limits (wheel)
    PRISMATIC,  // Slides (linear)
    FIXED,      // No movement
    FLOATING,   // 6DOF
    PLANAR      // Moves in a 2D plane
  };

  /// @brief Struct to hold joint limits of the robot model
  struct JointLimits
  {
    double min_position = -1e9;
    double max_position = 1e9;
    double max_velocity = 0.0;
    bool has_position_limits = false;
    bool has_velocity_limits = false;

    JointType jtype = JointType::UNKNOWN;
  };

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

    // Override of the initKinematics
    bool initKinematics(const std::string &urdf_xml, const std::string &base_frame,
                        const std::string &tool_frame) override;

    bool setCommand(const CommandVariant &command) override;

    std::vector<JointLimits> getJointLimits() const
    {
      return joint_limits_;
    }
    std::vector<std::string> getJointNames() const
    {
      return std::vector<std::string>(joint_names.begin(), joint_names.end());
    }

  private:
    std::vector<std::string> joint_names;

    std::vector<JointLimits> joint_limits_;
  }; // class GenericJointPosition
} // namespace robot_interfaces