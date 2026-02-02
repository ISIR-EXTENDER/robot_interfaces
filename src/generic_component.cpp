#include "robot_interfaces/generic_component.hpp"

#include "controller_interface/helpers.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <unordered_map>
#include <urdf/model.h>

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

  void GenericComponent::set_commands_names(std::vector<std::string> custom_names)
  {
    for (size_t i = 0; i < custom_names.size(); ++i)
    {
      command_names.emplace_back(custom_names[i]);
    }
  }

  std::vector<double> GenericComponent::get_states_values() const
  {
    std::vector<double> state_values;
    // Collect current values from all state interfaces
    for (const auto &state_interface : state_interfaces)
    {
      state_values.emplace_back(state_interface.get().get_value());
    }
    return state_values;
  }

  std::vector<double> GenericComponent::get_command_values() const
  {
    std::vector<double> command_values;
    // Collect current values from all command interfaces
    for (const auto &command_interface : command_interfaces)
    {
      command_values.emplace_back(command_interface.get().get_value());
    }

    return command_values;
  }

  bool GenericComponent::set_values(const std::vector<double> &values)
  {
    // Verify size matches before setting values
    if (values.size() != command_interfaces.size())
    {
      return false;
    }
    // Set each command interface with provided value
    for (size_t i = 0; i < command_interfaces.size(); ++i)
    {
      command_interfaces[i].get().set_value(values[i]);
    }

    return true;
  }

  // ==================== Kinematics Support (KDL-based) ====================

  bool GenericComponent::initKinematics(const std::string &urdf_xml, const std::string &base_frame,
                                        const std::string &tool_frame)
  {
    // Parse URDF
    if (!urdf_model.initString(urdf_xml))
    {
      RCLCPP_ERROR(rclcpp::get_logger(component_name), "URDF Parsing failed.");
      return false;
    }

    // Build KDL Chain
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree))
    {
      RCLCPP_ERROR(rclcpp::get_logger(component_name), "Failed to extract KDL tree from URDF.");
      return false;
    }

    if (!tree.getChain(base_frame, tool_frame, kdl_chain_))
    {
      RCLCPP_ERROR(rclcpp::get_logger(component_name), "Failed to find chain from %s to %s.",
                   base_frame.c_str(), tool_frame.c_str());
      return false;
    }

    // Initialize Solvers and Buffers
    unsigned int nj = kdl_chain_.getNrOfJoints();
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    kdl_joint_angles_.resize(nj);

    kdl_joint_to_interface_index_.clear();
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    {
      const auto &joint = kdl_chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None)
        continue; // Skip fixed joints

      auto it = std::find_if(state_names.begin(), state_names.end(), [&](const std::string &s) {
        return s.find(joint.getName()) != std::string::npos &&
               s.find("/position") != std::string::npos;
      });
      if (it == state_names.end())
      {
        RCLCPP_ERROR(rclcpp::get_logger(component_name), "Joint %s not in interfaces!",
                     joint.getName().c_str());
        return false;
      }
      kdl_joint_to_interface_index_.push_back(std::distance(state_names.begin(), it));
    }

    RCLCPP_INFO(rclcpp::get_logger(component_name), "Kinematics initialized for %u joints.", nj);
    return true;
  }

  CartesianPosition GenericComponent::computeFK() const
  {
    CartesianPosition pose;

    for (size_t i = 0; i < kdl_joint_to_interface_index_.size(); ++i)
    {
      size_t interface_idx = kdl_joint_to_interface_index_[i];
      kdl_joint_angles_(i) = state_interfaces[interface_idx].get().get_value();
    }

    KDL::Frame frame;
    if (fk_solver_->JntToCart(kdl_joint_angles_, frame) < 0)
    {
      return last_valid_pose_; // Fallback
    }

    pose.translation = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());

    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    pose.quaternion = Eigen::Quaterniond(w, x, y, z);

    // Ensure rotation doesn't "flip" (quaternion continuity)
    if (pose.quaternion.dot(last_valid_pose_.quaternion) < 0.0)
    {
      pose.quaternion.coeffs() *= -1.0;
    }

    last_valid_pose_ = pose;
    return pose;
  }

  CartesianPosition GenericComponent::getCurrentEndEffectorPose() const
  {
    // Return FK pose
    return computeFK();
  }
} // namespace robot_interfaces