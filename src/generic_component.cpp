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
    // 1. Parse URDF
    if (!urdf_model.initString(urdf_xml))
    {
      RCLCPP_ERROR(rclcpp::get_logger(component_name), "URDF Parsing failed.");
      return false;
    }

    // 2. Build KDL Chain
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

    // 3. Initialize Solvers and Buffers
    unsigned int nj = kdl_chain_.getNrOfJoints();
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    kdl_joint_angles_.resize(nj);

    // 4. Map KDL joints to Controller Interface indices
    // This removes the need for string-searching during the real-time loop.
    kdl_joint_to_interface_index_.clear();
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    {
      const auto &joint = kdl_chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None)
        continue; // Skip fixed joints

      auto it = std::find_if(state_names.begin(), state_names.end(), [&](const std::string &s) {
        return s.find(joint.getName()) != std::string::npos;
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

    // 1. Sync KDL joint angles from State Interfaces using our pre-built map
    for (size_t i = 0; i < kdl_joint_to_interface_index_.size(); ++i)
    {
      size_t interface_idx = kdl_joint_to_interface_index_[i];
      kdl_joint_angles_(i) = state_interfaces[interface_idx].get().get_value();
    }

    // 2. Compute Solve
    KDL::Frame frame;
    if (fk_solver_->JntToCart(kdl_joint_angles_, frame) < 0)
    {
      return last_valid_pose_; // Fallback
    }

    // 3. Convert KDL to Eigen/Pose
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

  // CartesianPosition GenericComponent::computeForwardKinematics() const
  // {
  //   // Initialize pose with zero translation and identity rotation
  //   CartesianPosition current_end_effector_pose;
  //   current_end_effector_pose.translation = Eigen::Vector3d::Zero();
  //   current_end_effector_pose.quaternion = Eigen::Quaterniond::Identity();

  //   // Lazy initialization: try to initialize KDL chain on first use
  //   if (!kdl_chain_initialized_ && !kdl_chain_initialization_attempted_)
  //   {
  //     kdl_chain_initialization_attempted_ = true;
  //     kdl_chain_initialized_ = initializeKDLChain();
  //   }

  //   // Verify forward kinematics solver is initialized
  //   if (!forward_kinematics_solver_)
  //   {
  //     // Fallback to cached pose if available; non-blocking behavior
  //     std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //     if (have_last_pose_)
  //     {
  //       return last_pose_;
  //     }
  //     return current_end_effector_pose;
  //   }

  //   try
  //   {
  //     // Lock joint state to ensure consistent read
  //     std::lock_guard<std::mutex> joint_state_mutex_lock(joint_state_access_mutex_);

  //     // Get the number of joints in the KDL chain (DOF)
  //     const size_t dof = kdl_chain_.getNrOfJoints();

  //     if (current_joint_positions_.size() < dof)
  //     {
  //       RCLCPP_WARN_THROTTLE(rclcpp::get_logger(component_name), *node_clock_obj_, 5000,
  //                            "Joint state incomplete: expected at least %zu joints, got %zu.",
  //                            dof, current_joint_positions_.size());
  //       // Fallback to cached pose if available
  //       std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //       if (have_last_pose_)
  //       {
  //         return last_pose_;
  //       }
  //       return current_end_effector_pose;
  //     }

  //     // Compute state age for deciding whether RT buffer is fresh
  //     double state_age_ms_for_rt = std::numeric_limits<double>::infinity();
  //     if (last_joint_state_update_.time_since_epoch().count() > 0)
  //     {
  //       const auto now = std::chrono::steady_clock::now();
  //       state_age_ms_for_rt =
  //           std::chrono::duration<double, std::milli>(now - last_joint_state_update_).count();
  //     }

  //     // Build KDL joint vector strictly by name-matching (ignore ordering and extra joints)
  //     size_t joint_index = 0;
  //     size_t matched_count = 0;
  //     std::unordered_map<std::string, double> name_to_pos;
  //     name_to_pos.reserve(current_joint_names_.size());
  //     for (size_t j = 0; j < current_joint_names_.size(); ++j)
  //     {
  //       if (j < current_joint_positions_.size())
  //       {
  //         name_to_pos[current_joint_names_[j]] = current_joint_positions_[j];
  //       }
  //     }

  //     for (size_t segment_index = 0; segment_index < kdl_chain_.getNrOfSegments();
  //     ++segment_index)
  //     {
  //       const KDL::Joint &segment_joint = kdl_chain_.getSegment(segment_index).getJoint();
  //       if (segment_joint.getType() == KDL::Joint::None)
  //       {
  //         continue; // skip fixed joints
  //       }
  //       const std::string &joint_name = segment_joint.getName();
  //       auto it = name_to_pos.find(joint_name);
  //       if (it == name_to_pos.end())
  //       {
  //         RCLCPP_ERROR(rclcpp::get_logger(component_name),
  //                      "FK aborted: required joint '%s' missing in joint_state.",
  //                      joint_name.c_str());
  //         std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //         if (have_last_pose_)
  //         {
  //           return last_pose_;
  //         }
  //         return current_end_effector_pose;
  //       }
  //       if (joint_index >= dof)
  //       {
  //         RCLCPP_ERROR(rclcpp::get_logger(component_name),
  //                      "FK internal error: joint_index overflow (index=%zu, dof=%zu)",
  //                      joint_index, dof);
  //         std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //         if (have_last_pose_)
  //         {
  //           return last_pose_;
  //         }
  //         return current_end_effector_pose;
  //       }
  //       kdl_joint_angles_(joint_index) = it->second;
  //       matched_count++;
  //       joint_index++;
  //     }

  //     if (matched_count != dof)
  //     {
  //       RCLCPP_ERROR(rclcpp::get_logger(component_name),
  //                    "FK aborted: matched %zu joints but KDL chain has %zu.", matched_count,
  //                    dof);
  //       std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //       if (have_last_pose_)
  //       {
  //         return last_pose_;
  //       }
  //       return current_end_effector_pose;
  //     }

  //     // Debug: log once the exact joint names and positions used for KDL FK (ordered by chain)
  //     if (fk_debug_logs_)
  //     {
  //       std::ostringstream oss;
  //       oss << "KDL FK joints used (ordered): ";
  //       size_t idx_out = 0;
  //       for (size_t segment_index = 0; segment_index < kdl_chain_.getNrOfSegments();
  //            ++segment_index)
  //       {
  //         const KDL::Joint &segment_joint = kdl_chain_.getSegment(segment_index).getJoint();
  //         if (segment_joint.getType() == KDL::Joint::None)
  //         {
  //           continue;
  //         }
  //         const std::string &jn = segment_joint.getName();
  //         const double val = (idx_out < dof) ? kdl_joint_angles_(idx_out) : 0.0;
  //         oss << jn << "=" << val;
  //         if (idx_out + 1 < dof)
  //         {
  //           oss << ", ";
  //         }
  //         idx_out++;
  //       }
  //       const std::string msg = oss.str();
  //       RCLCPP_INFO_ONCE(rclcpp::get_logger(component_name), "%s", msg.c_str());
  //     }

  //     KDL::Frame end_effector_frame_in_base;
  //     const int fk_computation_return_code =
  //         forward_kinematics_solver_->JntToCart(kdl_joint_angles_, end_effector_frame_in_base);

  //     if (fk_computation_return_code < 0)
  //     {
  //       RCLCPP_WARN_THROTTLE(rclcpp::get_logger(component_name), *node_clock_obj_, 5000,
  //                            "Forward kinematics computation failed with error code %d.",
  //                            fk_computation_return_code);
  //       std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //       if (have_last_pose_)
  //       {
  //         return last_pose_;
  //       }
  //       return current_end_effector_pose;
  //     }

  //     // Extract translation (position) from KDL frame
  //     current_end_effector_pose.translation =
  //         Eigen::Vector3d(end_effector_frame_in_base.p.x(), end_effector_frame_in_base.p.y(),
  //                         end_effector_frame_in_base.p.z());

  //     // Extract rotation (orientation) directly as quaternion and enforce hemisphere continuity
  //     double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  //     end_effector_frame_in_base.M.GetQuaternion(qx, qy, qz, qw);
  //     Eigen::Quaterniond q(qw, qx, qy, qz); // Eigen expects (w,x,y,z)
  //     q.normalize();

  //     {
  //       std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //       if (have_last_pose_)
  //       {
  //         if (q.dot(last_pose_.quaternion) < 0.0)
  //         {
  //           q.coeffs() *= -1.0; // keep same hemisphere to avoid flips
  //         }
  //       }
  //     }

  //     current_end_effector_pose.quaternion = q;

  //     // Log the computed pose for validation (shows pose changes with joint config changes)
  //     // Basic diagnostics: age of last joint state
  //     double state_age_ms = 0.0;
  //     if (last_joint_state_update_.time_since_epoch().count() > 0)
  //     {
  //       const auto now = std::chrono::steady_clock::now();
  //       state_age_ms =
  //           std::chrono::duration<double, std::milli>(now - last_joint_state_update_).count();
  //     }

  //     if (fk_debug_logs_)
  //     {
  //       RCLCPP_INFO_THROTTLE(
  //           rclcpp::get_logger(component_name), *node_clock_obj_, 2000,
  //           "FK Pose: pos=[%.4f, %.4f, %.4f] m | q=[w=%.4f, x=%.4f, y=%.4f, z=%.4f] | "
  //           "joint_state_age=%.1f ms",
  //           current_end_effector_pose.translation.x(), current_end_effector_pose.translation.y(),
  //           current_end_effector_pose.translation.z(), current_end_effector_pose.quaternion.w(),
  //           current_end_effector_pose.quaternion.x(), current_end_effector_pose.quaternion.y(),
  //           current_end_effector_pose.quaternion.z(), state_age_ms);
  //     }

  //     // Cache latest pose for non-blocking fallback
  //     {
  //       std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //       last_pose_ = current_end_effector_pose;
  //       have_last_pose_.store(true);
  //     }

  //     return current_end_effector_pose;
  //   }
  //   catch (const std::exception &exception_caught)
  //   {
  //     RCLCPP_WARN_THROTTLE(rclcpp::get_logger(component_name), *node_clock_obj_, 5000,
  //                          "Exception during forward kinematics computation: %s",
  //                          exception_caught.what());
  //     std::lock_guard<std::mutex> lock(last_pose_mutex_);
  //     if (have_last_pose_)
  //     {
  //       return last_pose_;
  //     }
  //     return current_end_effector_pose;
  //   }
  // }

} // namespace robot_interfaces