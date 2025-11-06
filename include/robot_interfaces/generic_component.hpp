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
  struct CartesianVelocityCommand
  {
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
  };
  struct CartesianPositionCommand
  {
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d translation;
  };

  using CommandVariant = std::variant<CartesianVelocityCommand, CartesianPositionCommand>;

  /// @brief Generic robot interfaces inspired by the franka approach and the
  /// semantic_component from ros2_control
  class GenericComponent
  {
  public:
    explicit GenericComponent(const std::string &name, size_t state_interface_size = 0,
                              size_t command_interface_size = 0);

    virtual ~GenericComponent() = default;

    bool assign_loaned_state(
        std::vector<hardware_interface::LoanedStateInterface> &state_interfaces);

    bool assign_loaned_command(
        std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces);

    void release_all_interfaces();

    virtual void set_states_names();

    virtual std::vector<std::string> get_states_names();

    virtual void set_commands_names();

    virtual std::vector<std::string> get_commands_names();

    std::vector<double> get_states_values() const;

    std::vector<double> get_command_values() const;

    bool set_values(const std::vector<double> &commanded_values);

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