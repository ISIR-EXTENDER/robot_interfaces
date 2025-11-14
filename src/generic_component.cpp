#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  GenericComponent::GenericComponent(const std::string &name, size_t state_interface_size,
                                     size_t command_interface_size)
      : component_name(name)
  {
    state_names.reserve(state_interface_size);
    command_names.reserve(command_interface_size);

    state_interfaces.reserve(state_interface_size);
    command_interfaces.reserve(command_interface_size);
  }

  bool GenericComponent::assign_loaned_state(
      std::vector<hardware_interface::LoanedStateInterface> &state_int)
  {
    return controller_interface::get_ordered_interfaces(state_int, state_names, "",
                                                        state_interfaces);
  }

  bool GenericComponent::assign_loaned_command(
      std::vector<hardware_interface::LoanedCommandInterface> &command_int)
  {
    return controller_interface::get_ordered_interfaces(command_int, command_names, "",
                                                        command_interfaces);
  }

  void GenericComponent::release_all_interfaces()
  {
    state_interfaces.clear();
    command_interfaces.clear();
  }

  void GenericComponent::set_states_names()
  {
    if (state_names.empty())
    {
      for (size_t i = 0; i < state_names.capacity(); ++i)
      {
        state_names.emplace_back(component_name + "/" + std::to_string(i + 1));
      }
    }
  }

  std::vector<std::string> GenericComponent::get_states_names()
  {
    return state_names;
  }

  void GenericComponent::set_commands_names()
  {
    if (command_names.empty())
    {
      for (size_t i = 0; i < command_names.capacity(); ++i)
      {
        command_names.emplace_back(component_name + "/" + std::to_string(i + 1));
      }
    }
  }

  std::vector<std::string> GenericComponent::get_commands_names()
  {
    return command_names;
  }

  std::vector<double> GenericComponent::get_states_values() const
  {
    std::vector<double> state_values;
    // insert all the state_interface_values
    for (const auto &state_interface : state_interfaces)
    {
      state_values.emplace_back(state_interface.get().get_value());
    }
    return state_values;
  }

  std::vector<double> GenericComponent::get_command_values() const
  {
    std::vector<double> command_values;
    // insert all the command_values
    for (const auto &command_interface : command_interfaces)
    {
      command_values.emplace_back(command_interface.get().get_value());
    }

    return command_values;
  }

  bool GenericComponent::set_values(const std::vector<double> &values)
  {
    // check we have sufficient memory
    if (values.capacity() != command_interfaces.size())
    {
      return false;
    }
    // // insert all the values
    for (size_t i = 0; i < command_interfaces.size(); ++i)
    {
      command_interfaces[i].get().set_value(values[i]);
    }

    return true;
  }
} // namespace robot_interfaces