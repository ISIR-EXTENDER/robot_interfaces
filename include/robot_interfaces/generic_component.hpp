#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>

// Hardware Interface
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

// RCLCPP (Logging only)
#include "rclcpp/rclcpp.hpp"

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

// URDF
#include <urdf/model.h>

#include "command_types.hpp"

namespace robot_interfaces
{
  /**
   * @brief Generic robot interfaces for ros2_control logic.
   * Handles interface management and KDL-based kinematics.
   */
  class GenericComponent
  {
  public:
    explicit GenericComponent(const std::string &name, 
                              size_t state_interface_size = 0,
                              size_t command_interface_size = 0);

    virtual ~GenericComponent() = default;

    // ==================== Interface Management ====================

    bool assign_loaned_state(std::vector<hardware_interface::LoanedStateInterface> &state_interfaces);
    bool assign_loaned_command(std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces);
    void release_all_interfaces();

    // ==================== Getters / Setters ====================

    virtual void set_states_names();
    virtual void set_commands_names(std::vector<std::string> custom_names = {});
    
    std::vector<std::string> get_states_names() const { return state_names; }
    std::vector<std::string> get_commands_names() const { return command_names; }

    std::vector<double> get_states_values() const;
    std::vector<double> get_command_values() const;
    bool set_values(const std::vector<double> &commanded_values);

    // ==================== Kinematics (Initialization) ====================

    /**
     * @brief High-level initialization for kinematics. 
     * Called during controller on_configure.
     */
    virtual bool initKinematics(const std::string &urdf_xml,
                        const std::string &base_frame,
                        const std::string &tool_frame);
    // ==================== Abstract Methods ====================

    virtual bool setCommand(const CommandVariant &command) = 0;
    CartesianPosition getCurrentEndEffectorPose() const;

  protected:
    // ==================== Core Math & Logic ====================

    /**
     * @brief Computes FK using current state interface values.
     * Real-time safe: no allocations, no mutexes, no strings.
     */
    CartesianPosition computeFK() const;

    // ==================== Data Members ====================
    std::string component_name;

    // Names used to "find" interfaces in the HardwareManager
    std::vector<std::string> state_names;
    std::vector<std::string> command_names;

    // Actual references to the hardware data
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces;

    // Kinematic structures
    urdf::Model urdf_model;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    
    // Frames
    std::string base_frame_name_ = "base_link";
    std::string tool_frame_name_ = "tool_frame";

    // Pre-allocated buffers for real-time loops
    mutable KDL::JntArray kdl_joint_angles_;
    mutable CartesianPosition last_valid_pose_;
    
    /**
     * @brief Maps KDL joint indices to state_interfaces indices.
     * index: KDL Joint Index -> value: state_interfaces index.
     */
    std::vector<size_t> kdl_joint_to_interface_index_;
  }; 
} // namespace robot_interfaces