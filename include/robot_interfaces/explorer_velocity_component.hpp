#pragma once

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <kdl/chainiksolvervel_wdls.hpp>

#include <Eigen/Dense>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  /**
   * @class ExplorerCartesianVelocity
   * @brief A specialized robot component for controlling explorer using the cartesian velocity
   * interface.
   *
   * This class inherits from `GenericComponent` and implements the specific logic required to
   * handle and set cartesian velocity commands for explorer.
   */
  class ExplorerCartesianVelocity : public GenericComponent
  {
  public:
    /**
     * @brief Construct a new Explorer Joint Position object.
     *
     * Initializes the component, setting up its command interface names and preparing it
     * to receive joint position commands.
     */
    explicit ExplorerCartesianVelocity();

    // Override of the initKinematics
    bool initKinematics(const std::string &urdf_xml, const std::string &base_frame,
                        const std::string &tool_frame) override;

    bool setCommand(const CommandVariant &command) override;

  private:
    const std::array<std::string, 6> joint_names{"joint_1", "joint_2", "joint_3",
                                                 "joint_4", "joint_5", "joint_6"};

    std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
    KDL::JntArray qdot_out_;
  }; // class ExplorerCartesianVelocity
} // namespace robot_interfaces