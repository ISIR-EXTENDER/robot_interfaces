#include "robot_interfaces/generic_component.hpp"
#include <atomic>
#include <fstream>
#include <kdl_parser/kdl_parser.hpp>
#include <unordered_map>
#include <sstream>
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

  void GenericComponent::onJointStateReceived(
      const sensor_msgs::msg::JointState::SharedPtr joint_state_message)
  {
    // Always capture raw message under mutex for diagnostics/fallback
    {
      std::lock_guard<std::mutex> joint_state_mutex_lock(joint_state_access_mutex_);
      current_joint_positions_ = joint_state_message->position;
      current_joint_names_ = joint_state_message->name;

      // Log first callback to verify message is received
      static bool first_callback = true;
      if (first_callback && fk_debug_logs_)
      {
        RCLCPP_INFO(rclcpp::get_logger(component_name),
                    "Joint state callback received: %zu joint names, %zu positions",
                    joint_state_message->name.size(), joint_state_message->position.size());
        first_callback = false;
      }
      // Mark readiness (first joint state observed)
      if (!have_first_joint_state_.load())
      {
        have_first_joint_state_.store(true);
      }
    }

    // Fill RT-friendly buffer by name-matching to KDL joint order (no index assumptions)
    if (use_rt_joint_buffer_ && rt_buffer_ready_ && kdl_chain_.getNrOfJoints() > 0)
    {
      const size_t dof = kdl_chain_.getNrOfJoints();
      if (rt_joint_positions_.size() != dof)
      {
        rt_joint_positions_.assign(dof, 0.0);
      }

      // Build name→position map from the incoming message
      std::unordered_map<std::string, double> name_to_pos;
      name_to_pos.reserve(joint_state_message->name.size());
      for (size_t j = 0; j < joint_state_message->name.size(); ++j)
      {
        if (j < joint_state_message->position.size())
        {
          name_to_pos[joint_state_message->name[j]] = joint_state_message->position[j];
        }
      }

      size_t joint_index = 0;
        for (size_t segment_index = 0; segment_index < kdl_chain_.getNrOfSegments(); ++segment_index)
      {
        const KDL::Joint &segment_joint = kdl_chain_.getSegment(segment_index).getJoint();
        if (segment_joint.getType() == KDL::Joint::None)
        {
          continue; // skip fixed joints
        }
        const std::string &joint_name = segment_joint.getName();
        auto it = name_to_pos.find(joint_name);
        if (it != name_to_pos.end())
        {
          if (joint_index < rt_joint_positions_.size())
          {
            rt_joint_positions_[joint_index] = it->second;
          }
        }
        else if (fk_debug_logs_)
        {
          RCLCPP_WARN_THROTTLE(rclcpp::get_logger(component_name), *node_clock_obj_, 5000,
                               "RT buffer: joint '%s' missing in joint_state; leaving previous value.",
                               joint_name.c_str());
        }
        joint_index++;
      }
    }
    last_joint_state_update_ = std::chrono::steady_clock::now();
  }

  void GenericComponent::initializeJointStateSubscription() const
  {
    if (joint_state_subscription_initialized_ || !node_base_ || !node_topics_)
    {
      return; // Already initialized or node not available
    }

    try
    {
      const auto joint_state_qos = rclcpp::SensorDataQoS();

      // Cast away const to modify mutable members in const context
      auto *self = const_cast<GenericComponent *>(this);

      // Try to get lifecycle node temporarily to create subscription
      // If not available, will try on next call
      auto node = lifecycle_node_.lock();
      if (!node)
      {
        return;
      }

      // Use LifecycleNode to create subscription (processed by controller_manager's executor)
      self->joint_state_subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
          joint_states_topic_, joint_state_qos,
          std::bind(&GenericComponent::onJointStateReceived, self, std::placeholders::_1));

      // Note: Subscriptions will be handled by the controller manager's executor
      // No separate spinning thread needed

      self->joint_state_subscription_initialized_ = true;
      RCLCPP_INFO(rclcpp::get_logger(component_name),
                  "Joint state subscription initialized successfully (topic: %s).",
                  joint_states_topic_.c_str());
    }
    catch (const std::exception &exception_caught)
    {
      RCLCPP_WARN(rclcpp::get_logger(component_name),
                  "Failed to initialize joint state subscription: %s", exception_caught.what());
    }
  }

  void GenericComponent::setNodeInterfaces(rclcpp::Node::SharedPtr node)
  {
    if (!node)
    {
      return;
    }
    node_base_ = node->get_node_base_interface();
    node_topics_ = node->get_node_topics_interface();
    node_logging_ = node->get_node_logging_interface();
    node_clock_ = node->get_node_clock_interface();
    node_clock_obj_ = node->get_clock();
    // Optional parameters
    try
    {
      if (node->has_parameter("joint_states_topic"))
      {
        joint_states_topic_ = node->get_parameter("joint_states_topic").as_string();
      }
      else
      {
        RCLCPP_INFO_ONCE(rclcpp::get_logger(component_name),
                         "Using default joint states topic: '%s'", joint_states_topic_.c_str());
      }
      if (node->has_parameter("use_rt_joint_buffer"))
      {
        use_rt_joint_buffer_ = node->get_parameter("use_rt_joint_buffer").as_bool();
      }
    }
    catch (...)
    {
    }
    // Create subscriptions immediately
    initializeRobotDescriptionSubscription();
    initializeJointStateSubscription();
  }

  void GenericComponent::setNodeInterfaces(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    if (!node)
    {
      return;
    }
    // Extract and store only the interfaces, not a shared_ptr to the full node
    node_base_ = node->get_node_base_interface();
    node_topics_ = node->get_node_topics_interface();
    node_logging_ = node->get_node_logging_interface();
    node_clock_ = node->get_node_clock_interface();
    node_clock_obj_ = node->get_clock();

    // Optional: store weak reference for diagnostics only (doesn't keep node alive)
    lifecycle_node_ = node;
    // Optional parameters
    try
    {
      if (node->has_parameter("joint_states_topic"))
      {
        joint_states_topic_ = node->get_parameter("joint_states_topic").as_string();
      }
      else
      {
        RCLCPP_INFO_ONCE(rclcpp::get_logger(component_name),
                         "Using default joint states topic: '%s'", joint_states_topic_.c_str());
      }
      if (node->has_parameter("use_rt_joint_buffer"))
      {
        use_rt_joint_buffer_ = node->get_parameter("use_rt_joint_buffer").as_bool();
      }
    }
    catch (...)
    {
    }
    // Create subscriptions immediately
    initializeRobotDescriptionSubscription();
    initializeJointStateSubscription();
  }

  void GenericComponent::initializeRobotDescriptionSubscription() const
  {
    if (robot_description_subscription_initialized_ || !node_base_ || !node_topics_)
    {
      return; // Already initialized or node not available
    }

    try
    {
      const auto string_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

      // Cast away const to modify mutable members in const context
      auto *self = const_cast<GenericComponent *>(this);

      // Try to get lifecycle node temporarily to create subscription
      // If not available, will try on next call
      auto node = lifecycle_node_.lock();
      if (!node)
      {
        return;
      }

      // Subscribe to /robot_description topic using LifecycleNode
      self->robot_description_subscription_ = node->create_subscription<std_msgs::msg::String>(
          "/robot_description", string_qos,
          std::bind(&GenericComponent::onRobotDescriptionReceived, self, std::placeholders::_1));

      self->robot_description_subscription_initialized_ = true;
      RCLCPP_INFO(rclcpp::get_logger(component_name),
                  "Robot description subscription initialized successfully.");
    }
    catch (const std::exception &exception_caught)
    {
      RCLCPP_WARN(rclcpp::get_logger(component_name),
                  "Failed to initialize robot_description subscription: %s",
                  exception_caught.what());
    }
  }

  void GenericComponent::onRobotDescriptionReceived(const std_msgs::msg::String::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(robot_description_mutex_);
      robot_description_xml_ = msg->data;
    }
    have_robot_description_.store(true);
    RCLCPP_INFO_ONCE(rclcpp::get_logger(component_name),
                     "Received robot_description from /robot_description topic (%zu bytes).",
                     robot_description_xml_.size());
  }

  void GenericComponent::setRobotDescription(const std::string &robot_description_xml)
  {
    robot_description_xml_ = robot_description_xml;
  }

  void GenericComponent::setFrameNames(const std::string &base_frame, const std::string &tool_frame)
  {
    base_frame_name_ = base_frame;
    tool_frame_name_ = tool_frame;
  }

  bool GenericComponent::initializeKDLChain() const
  {
    try
    {
      if (!node_base_ || !node_topics_)
      {
        RCLCPP_ERROR(rclcpp::get_logger(component_name),
                     "Node interfaces not initialized yet; KDL chain initialization deferred.");
        return false;
      }
      // Non-blocking: fetch robot_description once; if empty, skip
      std::string robot_description_xml;
      {
        std::lock_guard<std::mutex> lock(robot_description_mutex_);
        robot_description_xml = robot_description_xml_;
      }
      if (robot_description_xml.empty())
      {
        RCLCPP_WARN(rclcpp::get_logger(component_name),
                    "robot_description not available; KDL init skipped.");
        return false;
      }

      // Parse URDF
      urdf::Model urdf_model;
      if (!urdf_model.initString(robot_description_xml))
      {
        RCLCPP_ERROR(rclcpp::get_logger(component_name),
                     "Failed to parse URDF from robot_description.");
        return false;
      }

      // Build KDL tree from URDF
      KDL::Tree kdl_tree;
      if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
      {
        RCLCPP_ERROR(rclcpp::get_logger(component_name), "Failed to build KDL tree from URDF.");
        return false;
      }

      // Get frame names from member variables (set via setFrameNames)
      std::string base_frame = base_frame_name_;
      std::string tool_frame = tool_frame_name_;

      // Extract kinematic chain between frames
      if (!kdl_tree.getChain(base_frame, tool_frame, kdl_chain_))
      {
        RCLCPP_ERROR(rclcpp::get_logger(component_name),
                     "Failed to extract KDL chain from '%s' to '%s'. "
                     "Verify these frames exist in URDF.",
                     base_frame.c_str(), tool_frame.c_str());
        return false;
      }

      // Initialize forward kinematics solver
      forward_kinematics_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
      // Preallocate joint array for FK
      kdl_joint_angles_.resize(kdl_chain_.getNrOfJoints());

      // Verify joint name alignment once, if joint states are already available
      verifyJointNameAlignment();

      // Allocate RT-friendly buffer sized to DOF
      rt_joint_positions_.assign(kdl_chain_.getNrOfJoints(), 0.0);
      kdl_joint_angles_.resize(kdl_chain_.getNrOfJoints());
      // Only mark RT buffer ready if we intend to use/update it.
      rt_buffer_ready_ = use_rt_joint_buffer_;

      if (fk_debug_logs_)
      {
        RCLCPP_INFO(rclcpp::get_logger(component_name),
                    "Forward kinematics initialized: '%s' -> '%s' (%u DOF, %u segments)",
                    base_frame.c_str(), tool_frame.c_str(), kdl_chain_.getNrOfJoints(),
                    kdl_chain_.getNrOfSegments());
      }

      kdl_ready_.store(true);
      return true;
    }
    catch (const std::exception &exception_caught)
    {
      RCLCPP_ERROR(rclcpp::get_logger(component_name),
                   "Exception during KDL chain initialization: %s", exception_caught.what());
      return false;
    }
  }

  void GenericComponent::verifyJointNameAlignment() const
  {
    if (joint_alignment_verified_)
    {
      return;
    }
    std::lock_guard<std::mutex> joint_state_mutex_lock(joint_state_access_mutex_);
    if (current_joint_names_.empty())
    {
      // No joint states yet; skip for now
      return;
    }

    // Collect expected joint names from KDL chain (skip fixed joints)
    std::vector<std::string> expected_joint_names;
    expected_joint_names.reserve(kdl_chain_.getNrOfJoints());
    for (size_t segment_index = 0; segment_index < kdl_chain_.getNrOfSegments(); ++segment_index)
    {
      const KDL::Joint &segment_joint = kdl_chain_.getSegment(segment_index).getJoint();
      if (segment_joint.getType() == KDL::Joint::None)
      {
        continue;
      }
      expected_joint_names.emplace_back(segment_joint.getName());
    }

    // Build mapping from KDL joint order to joint_state indices
    kdl_to_state_index_.clear();
    kdl_to_state_index_.reserve(expected_joint_names.size());
    bool names_match = true;
    for (const auto &expected_name : expected_joint_names)
    {
      auto it = std::find(current_joint_names_.begin(), current_joint_names_.end(), expected_name);
      if (it == current_joint_names_.end())
      {
        names_match = false;
        RCLCPP_WARN(rclcpp::get_logger(component_name),
                    "Expected joint '%s' not found in current joint states.",
                    expected_name.c_str());
        kdl_to_state_index_.push_back(-1);
      }
      else
      {
        kdl_to_state_index_.push_back(
            static_cast<int>(std::distance(current_joint_names_.begin(), it)));
      }
    }

    if (!names_match)
    {
      RCLCPP_WARN(rclcpp::get_logger(component_name),
                  "Joint name mismatch detected between KDL chain and joint states. "
                  "Ensure joint_state_broadcaster publishes names matching URDF.");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(component_name),
                  "Joint names align with KDL chain (%zu joints).", expected_joint_names.size());
    }

    joint_alignment_verified_ = true;
  }

  CartesianPosition GenericComponent::computeForwardKinematics() const
  {
    // Initialize pose with zero translation and identity rotation
    CartesianPosition current_end_effector_pose;
    current_end_effector_pose.translation = Eigen::Vector3d::Zero();
    current_end_effector_pose.quaternion = Eigen::Quaterniond::Identity();

    // Lazy initialization: try to initialize KDL chain on first use
    if (!kdl_chain_initialized_ && !kdl_chain_initialization_attempted_)
    {
      kdl_chain_initialization_attempted_ = true;
      kdl_chain_initialized_ = initializeKDLChain();
    }

    // Verify forward kinematics solver is initialized
    if (!forward_kinematics_solver_)
    {
      // Fallback to cached pose if available; non-blocking behavior
      std::lock_guard<std::mutex> lock(last_pose_mutex_);
      if (have_last_pose_)
      {
        return last_pose_;
      }
      return current_end_effector_pose;
    }

    try
    {
      // Lock joint state to ensure consistent read
      std::lock_guard<std::mutex> joint_state_mutex_lock(joint_state_access_mutex_);

      // Get the number of joints in the KDL chain (DOF)
      const size_t dof = kdl_chain_.getNrOfJoints();

      if (current_joint_positions_.size() < dof)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger(component_name), *node_clock_obj_, 5000,
                             "Joint state incomplete: expected at least %zu joints, got %zu.", dof,
                             current_joint_positions_.size());
        // Fallback to cached pose if available
        std::lock_guard<std::mutex> lock(last_pose_mutex_);
        if (have_last_pose_)
        {
          return last_pose_;
        }
        return current_end_effector_pose;
      }

      // Compute state age for deciding whether RT buffer is fresh
      double state_age_ms_for_rt = std::numeric_limits<double>::infinity();
      if (last_joint_state_update_.time_since_epoch().count() > 0)
      {
        const auto now = std::chrono::steady_clock::now();
        state_age_ms_for_rt =
            std::chrono::duration<double, std::milli>(now - last_joint_state_update_).count();
      }

      // Build KDL joint vector strictly by name-matching (ignore ordering and extra joints)
      size_t joint_index = 0;
      size_t matched_count = 0;
      std::unordered_map<std::string, double> name_to_pos;
      name_to_pos.reserve(current_joint_names_.size());
      for (size_t j = 0; j < current_joint_names_.size(); ++j)
      {
        if (j < current_joint_positions_.size())
        {
          name_to_pos[current_joint_names_[j]] = current_joint_positions_[j];
        }
      }

      for (size_t segment_index = 0; segment_index < kdl_chain_.getNrOfSegments(); ++segment_index)
      {
        const KDL::Joint &segment_joint = kdl_chain_.getSegment(segment_index).getJoint();
        if (segment_joint.getType() == KDL::Joint::None)
        {
          continue; // skip fixed joints
        }
        const std::string &joint_name = segment_joint.getName();
        auto it = name_to_pos.find(joint_name);
        if (it == name_to_pos.end())
        {
          RCLCPP_ERROR(rclcpp::get_logger(component_name),
                       "FK aborted: required joint '%s' missing in joint_state.",
                       joint_name.c_str());
          std::lock_guard<std::mutex> lock(last_pose_mutex_);
          if (have_last_pose_)
          {
            return last_pose_;
          }
          return current_end_effector_pose;
        }
        if (joint_index >= dof)
        {
          RCLCPP_ERROR(rclcpp::get_logger(component_name),
                       "FK internal error: joint_index overflow (index=%zu, dof=%zu)",
                       joint_index, dof);
          std::lock_guard<std::mutex> lock(last_pose_mutex_);
          if (have_last_pose_)
          {
            return last_pose_;
          }
          return current_end_effector_pose;
        }
        kdl_joint_angles_(joint_index) = it->second;
        matched_count++;
        joint_index++;
      }

      if (matched_count != dof)
      {
        RCLCPP_ERROR(rclcpp::get_logger(component_name),
                     "FK aborted: matched %zu joints but KDL chain has %zu.", matched_count, dof);
        std::lock_guard<std::mutex> lock(last_pose_mutex_);
        if (have_last_pose_)
        {
          return last_pose_;
        }
        return current_end_effector_pose;
      }

      // Debug: log once the exact joint names and positions used for KDL FK (ordered by chain)
      if (fk_debug_logs_)
      {
        std::ostringstream oss;
        oss << "KDL FK joints used (ordered): ";
        size_t idx_out = 0;
        for (size_t segment_index = 0; segment_index < kdl_chain_.getNrOfSegments(); ++segment_index)
        {
          const KDL::Joint &segment_joint = kdl_chain_.getSegment(segment_index).getJoint();
          if (segment_joint.getType() == KDL::Joint::None)
          {
            continue;
          }
          const std::string &jn = segment_joint.getName();
          const double val = (idx_out < dof) ? kdl_joint_angles_(idx_out) : 0.0;
          oss << jn << "=" << val;
          if (idx_out + 1 < dof)
          {
            oss << ", ";
          }
          idx_out++;
        }
        const std::string msg = oss.str();
        RCLCPP_INFO_ONCE(rclcpp::get_logger(component_name), "%s", msg.c_str());
      }

      KDL::Frame end_effector_frame_in_base;
      const int fk_computation_return_code =
          forward_kinematics_solver_->JntToCart(kdl_joint_angles_, end_effector_frame_in_base);

      if (fk_computation_return_code < 0)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger(component_name), *node_clock_obj_, 5000,
                             "Forward kinematics computation failed with error code %d.",
                             fk_computation_return_code);
        std::lock_guard<std::mutex> lock(last_pose_mutex_);
        if (have_last_pose_)
        {
          return last_pose_;
        }
        return current_end_effector_pose;
      }

      // Extract translation (position) from KDL frame
      current_end_effector_pose.translation =
          Eigen::Vector3d(end_effector_frame_in_base.p.x(), end_effector_frame_in_base.p.y(),
                          end_effector_frame_in_base.p.z());

      // Extract rotation (orientation) directly as quaternion and enforce hemisphere continuity
      double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
      end_effector_frame_in_base.M.GetQuaternion(qx, qy, qz, qw);
      Eigen::Quaterniond q(qw, qx, qy, qz); // Eigen expects (w,x,y,z)
      q.normalize();

      {
        std::lock_guard<std::mutex> lock(last_pose_mutex_);
        if (have_last_pose_)
        {
          if (q.dot(last_pose_.quaternion) < 0.0)
          {
            q.coeffs() *= -1.0; // keep same hemisphere to avoid flips
          }
        }
      }

      current_end_effector_pose.quaternion = q;

      // Log the computed pose for validation (shows pose changes with joint config changes)
      // Basic diagnostics: age of last joint state
      double state_age_ms = 0.0;
      if (last_joint_state_update_.time_since_epoch().count() > 0)
      {
        const auto now = std::chrono::steady_clock::now();
        state_age_ms =
            std::chrono::duration<double, std::milli>(now - last_joint_state_update_).count();
      }

      if (fk_debug_logs_)
      {
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger(component_name), *node_clock_obj_, 2000,
            "FK Pose: pos=[%.4f, %.4f, %.4f] m | q=[w=%.4f, x=%.4f, y=%.4f, z=%.4f] | "
            "joint_state_age=%.1f ms",
            current_end_effector_pose.translation.x(), current_end_effector_pose.translation.y(),
            current_end_effector_pose.translation.z(), current_end_effector_pose.quaternion.w(),
            current_end_effector_pose.quaternion.x(), current_end_effector_pose.quaternion.y(),
            current_end_effector_pose.quaternion.z(), state_age_ms);
      }

      // Cache latest pose for non-blocking fallback
      {
        std::lock_guard<std::mutex> lock(last_pose_mutex_);
        last_pose_ = current_end_effector_pose;
        have_last_pose_.store(true);
      }

      return current_end_effector_pose;
    }
    catch (const std::exception &exception_caught)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(component_name), *node_clock_obj_, 5000,
                           "Exception during forward kinematics computation: %s",
                           exception_caught.what());
      std::lock_guard<std::mutex> lock(last_pose_mutex_);
      if (have_last_pose_)
      {
        return last_pose_;
      }
      return current_end_effector_pose;
    }
  }

} // namespace robot_interfaces