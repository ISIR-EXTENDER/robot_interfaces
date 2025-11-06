#include "robot_interfaces/robot_interfaces_algos.hpp"

namespace robot_interfaces
{
  std::unique_ptr<GenericComponent> create_robot_component(const std::string &robot_type)
  {
    if (robot_type == "franka_velocity")
    {
      return std::make_unique<FrankaCartesianVelocity>();
    }
    else
    {
      return nullptr;
    }
  }
} // namespace robot_interfaces
