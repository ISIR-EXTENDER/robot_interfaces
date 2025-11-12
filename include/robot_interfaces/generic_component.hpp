#pragma once

#include <string>
#include <variant>
#include <vector>

#include <Eigen/Dense>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "controller_interface/helpers.hpp"

namespace robot_interfaces
{
  /// @brief Generic cartesian velocity command for a manipulator end effector
  struct CartesianVelocityCommand
  {
    /// @brief Linear velocity (x, y, z)
    Eigen::Vector3d linear;
    /// @brief Angular velocity (roll, pitch, yaw)
    Eigen::Vector3d angular;
  };
  /// @brief Generic cartesian position command for a manipulator end effector
  struct CartesianPositionCommand
  {
    /// @brief Target position (x, y, z)
    Eigen::Vector3d translation;
    /// @brief Target orientation as a quaternion
    Eigen::Quaterniond quaternion;
  };
  /// @brief A variant to hold all the possible commands
  using CommandVariant = std::variant<CartesianVelocityCommand, CartesianPositionCommand>;

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
    virtual bool setCommand(const CommandVariant &command);

  protected:
    std::string component_name;

    std::vector<std::string> state_names;

    std::vector<std::string> command_names;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        command_interfaces;
  }; // class GenericComponent
} // namespace robot_interfaces