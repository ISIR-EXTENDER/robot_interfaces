#include "robot_interfaces/robot_interfaces_algos.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_interfaces
{
  std::unique_ptr<GenericComponent> create_robot_component(const std::string &robot_type)
  {
    if (robot_type == "franka_velocity")
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotInterfaceFactory"),
                  "Creating FrankaCartesianVelocity for robot_type='%s'", robot_type.c_str());
      return std::make_unique<FrankaCartesianVelocity>();
    }
    else if (robot_type == "kinova_velocity")
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotInterfaceFactory"),
                  "Creating KinovaCartesianVelocity for robot_type='%s'", robot_type.c_str());
      return std::make_unique<KinovaCartesianVelocity>();
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotInterfaceFactory"),
                   "Unknown robot_type '%s' in create_robot_component", robot_type.c_str());
      return nullptr;
    }
  }
} // namespace robot_interfaces
