#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "robot_interfaces/franka_velocity_component.hpp"
#include "robot_interfaces/kinova_velocity_component.hpp"
#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  std::unique_ptr<GenericComponent> create_robot_component(const std::string &robot_type);
} // namespace robot_interfaces