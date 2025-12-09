#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "controller_interface/helpers.hpp"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/create_subscription.hpp>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

namespace robot_interfaces
{
  /// @brief Generic cartesian velocity for a manipulator end effector
  struct CartesianVelocity
  {
    /// @brief Linear velocity (x, y, z)
    Eigen::Vector3d linear;
    /// @brief Angular velocity (roll, pitch, yaw)
    Eigen::Vector3d angular;
  };
  /// @brief Generic cartesian position for a manipulator end effector
  struct CartesianPosition
  {
    /// @brief Target position (x, y, z)
    Eigen::Vector3d translation;
    /// @brief Target orientation as a quaternion
    Eigen::Quaterniond quaternion;
  };

  /// @brief Generic joint command. Can be used for position, velocity or effort.
  struct JointCommand
  {
    /// @brief Vector of command. Must be the same size as the command interface vector
    std::vector<double> command;
  };

  /// @brief A variant to hold all the possible commands
  using CommandVariant = std::variant<CartesianVelocity, CartesianPosition, JointCommand>;

  /// @brief Generic robot interfaces inspired by the franka approach and the
  /// semantic_component from ros2_control
  class GenericComponent
  {
  public:
    explicit GenericComponent(const std::string &name, size_t state_interface_size = 0,
                              size_t command_interface_size = 0);

    virtual ~GenericComponent() = default;

    /**
     * @brief Assigns loaned state interfaces to this component.
     *
     * @param state_interfaces A vector of loaned state interfaces from the resource manager.
     * @return true if the assignment is successful, false otherwise.
     */
    bool assign_loaned_state(
        std::vector<hardware_interface::LoanedStateInterface> &state_interfaces);

    /**
     * @brief Assigns loaned command interfaces to this component.
     *
     * @param command_interfaces A vector of loaned command interfaces from the resource manager.
     * @return true if the assignment is successful, false otherwise.
     */
    bool assign_loaned_command(
        std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces);

    /**
     * @brief Releases all currently held state and command interfaces.
     */
    void release_all_interfaces();

    /**
     * @brief Virtual function to set the names of the state interfaces.
     */
    virtual void set_states_names();

    /**
     * @brief Gets the names of the state interfaces.
     *
     * @return A vector of strings containing the names of the state interfaces.
     */
    virtual std::vector<std::string> get_states_names();

    /**
     * @brief Virtual function to set the names of the command interfaces.
     */
    virtual void set_commands_names();

    /**
     * @brief Gets the names of the command interfaces.
     *
     * @return A vector of strings containing the names of the command interfaces.
     */
    virtual std::vector<std::string> get_commands_names();

    /**
     * @brief Retrieves the current values from all assigned state interfaces.
     *
     * @return A vector of doubles representing the current state values in the order of
     * state_names.
     */
    std::vector<double> get_states_values() const;

    /**
     * @brief Retrieves the current values from all assigned command interfaces.
     *
     * @return A vector of doubles representing the current command values in the order of
     * command_names.
     */
    std::vector<double> get_command_values() const;

    /**
     * @brief Sets the values for the command interfaces.
     *
     * @param commanded_values A vector of doubles containing the values to be commanded.
     * The size must match the number of command interfaces.
     * @return true if the values were set successfully, false otherwise.
     */
    bool set_values(const std::vector<double> &commanded_values);

    /**
     * @brief Sets a command from a CommandVariant type.
     *
     * This virtual function has to be overridden by derived classes to handle specific
     * high-level command types like Cartesian velocity or position.
     * It should call set_values, with the command variant, successfully transformed into a
     * std::vector<double>
     *
     * @param command The command to be set, contained within a std::variant.
     * @return true if the command was successfully processed, false otherwise.
     */
    virtual bool setCommand(const CommandVariant &command) = 0;

    /**
     * @brief Get pose of the end effector.
     *
     * This virtual function has to be overridden by derived classes to handle specific
     * hardware interface for each robot.
     *
     */
    virtual CartesianPosition getCurrentEndEffectorPose() const = 0;

    /**
     * @brief Callback for joint state updates.
     * Automatically stores positions and names for FK computation.
     * Made public to allow binding in derived class constructors.
     */
    void onJointStateReceived(const sensor_msgs::msg::JointState::SharedPtr joint_state_message);

    /**
     * @brief Set the base frame and tool frame names for KDL chain extraction.
     * Must be called before getCurrentEndEffectorPose() for correct FK computation.
     *
     * @param base_frame Name of the base frame in the URDF (e.g., "gen3_base_link")
     * @param tool_frame Name of the tool/end-effector frame in the URDF (e.g.,
     * "gen3_end_effector_link")
     */
    void setFrameNames(const std::string &base_frame, const std::string &tool_frame);

    /**
     * @brief Set the robot_description URDF string directly.
     * Must be called before getCurrentEndEffectorPose() to enable forward kinematics.
     *
     * @param robot_description_xml The URDF robot description as a string
     */
    void setRobotDescription(const std::string &robot_description_xml);

    /**
     * @brief Set the ROS 2 node interfaces for parameter access and topic subscriptions.
     * Supports both rclcpp::Node and rclcpp_lifecycle::LifecycleNode.
     *
     * @param node The ROS 2 node to extract interfaces from
     */
    void setNodeInterfaces(rclcpp::Node::SharedPtr node);
    void setNodeInterfaces(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  protected:
    // ==================== Generic Members ====================
    std::string component_name;

    std::vector<std::string> state_names;

    std::vector<std::string> command_names;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        command_interfaces;

    // ==================== Kinematics Support (KDL-based) ====================

    /**
     * @brief Initialize KDL chain and FK solver (lazy initialization).
     * Called automatically on first FK query. Reads frame names from ROS 2 parameters.
     * @return true if initialization successful, false otherwise
     */
    bool initializeKDLChain() const;

    /**
     * @brief Verify mapping between KDL chain joints and current joint state names.
     * Logs diagnostic differences to aid configuration issues.
     */
    void verifyJointNameAlignment() const;

    /**
     * @brief Initialize robot_description subscription (lazy initialization).
     * Subscribes to the /robot_description topic published by robot_state_publisher.
     */
    void initializeRobotDescriptionSubscription() const;

    /**
     * @brief Callback for robot_description updates from the /robot_description topic.
     * Automatically stores the URDF for FK chain initialization.
     */
    void onRobotDescriptionReceived(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief Initialize joint state subscription (lazy initialization).
     * Called once during first FK chain initialization.
     */
    void initializeJointStateSubscription() const;

    /**
     * @brief Compute forward kinematics for current joint positions.
     * Requires KDL chain to be initialized.
     * @return CartesianPosition with current end-effector pose
     */
    CartesianPosition computeForwardKinematics() const;

    // ==================== Readiness Flags and Cached Pose ====================
    // Readiness flags
    mutable std::atomic<bool> have_robot_description_{false};
    mutable std::atomic<bool> have_first_joint_state_{false};
    mutable std::atomic<bool> kdl_ready_{false};

    // Cached last pose
    mutable std::mutex last_pose_mutex_;
    mutable CartesianPosition last_pose_;
    mutable std::atomic<bool> have_last_pose_{false};

    // ROS 2 node interfaces for subscriptions and logging (extracted from controller's node)
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
    rclcpp::Clock::SharedPtr node_clock_obj_;

    // Optional weak reference to LifecycleNode (for diagnostics only, not kept alive)
    mutable std::weak_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

    // Joint states topic (configurable via parameter)
    std::string joint_states_topic_ = "/joint_states";

    // Robot description URDF (thread-safe)
    mutable std::mutex robot_description_mutex_;
    std::string robot_description_xml_;

    // Frame names for KDL chain extraction
    std::string base_frame_name_ = "base_link";
    std::string tool_frame_name_ = "tool_frame";

    // Joint state management (thread-safe)
    mutable std::mutex joint_state_access_mutex_;
    std::vector<double> current_joint_positions_;
    std::vector<std::string> current_joint_names_;
    mutable std::chrono::steady_clock::time_point last_joint_state_update_{};
    mutable std::vector<int>
        kdl_to_state_index_; // mapping from KDL joint order to joint_state indices
    mutable std::vector<double> rt_joint_positions_; // preallocated buffer sized to DOF
    mutable std::atomic<bool> rt_buffer_ready_{false};
    // Controls whether the RT-friendly joint buffer is used and maintained.
    // When false (default), FK reads directly from current_joint_positions_.
    bool use_rt_joint_buffer_{false};

    // KDL forward kinematics solver
    mutable KDL::Chain kdl_chain_;
    mutable std::unique_ptr<KDL::ChainFkSolverPos_recursive> forward_kinematics_solver_;
    mutable KDL::JntArray kdl_joint_angles_;
    mutable bool kdl_chain_initialized_{false};
    mutable bool kdl_chain_initialization_attempted_{false};
    mutable bool robot_description_subscription_initialized_{false}; // Lazy subscription init
    mutable bool joint_state_subscription_initialized_{false};       // Lazy subscription init
    // Removed all waits/timeouts to keep controller-safe, non-blocking behavior

    // Diagnostics configuration
    mutable bool joint_alignment_verified_{false};
    mutable bool fk_debug_logs_{true};
  }; // class GenericComponent
} // namespace robot_interfaces