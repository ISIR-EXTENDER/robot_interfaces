#include "robot_interfaces/generic_component.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>
// KDL and URDF parsing for extracting expected joint names from the chain
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace robot_interfaces
{
  // Simple concrete implementation for testing
  class TestFKComponent : public GenericComponent
  {
  public:
    TestFKComponent() : GenericComponent("test_fk_component", 0, 0)
    {
    }

    bool setCommand(const CommandVariant & /*command*/) override
    {
      return true; // Not used in FK test
    }

    CartesianPosition getCurrentEndEffectorPose() const override
    {
      return computeForwardKinematics();
    }

    // Expose these for testing
    void setNodeForTesting(const std::shared_ptr<rclcpp::Node> &node)
    {
      if (node)
      {
        node_base_ = node->get_node_base_interface();
        node_topics_ = node->get_node_topics_interface();
        node_logging_ = node->get_node_logging_interface();
        node_clock_ = node->get_node_clock_interface();
        node_clock_obj_ = node->get_clock();
      }
    }

    void injectJointState(const std::vector<double> &positions,
                          const std::string &joint_prefix = "gen3_joint_", int num_joints = 7)
    {
      auto msg = std::make_shared<sensor_msgs::msg::JointState>();
      msg->header.stamp.sec = 0;
      msg->header.stamp.nanosec = 0;

      msg->name.clear();
      if (!expected_joint_names_.empty())
      {
        msg->name = expected_joint_names_;
      }
      else
      {
        for (int i = 1; i <= num_joints; ++i)
        {
          msg->name.push_back(joint_prefix + std::to_string(i));
        }
      }

      msg->position = positions;
      msg->velocity = std::vector<double>(num_joints, 0.0);
      msg->effort = std::vector<double>(num_joints, 0.0);
      onJointStateReceived(msg);
    }

    // Populate expected joint names from KDL chain built from URDF
    void setExpectedJointNamesFromURDF(const std::string &urdf_xml, const std::string &base_frame,
                                       const std::string &tool_frame)
    {
      expected_joint_names_.clear();
      try
      {
        urdf::Model urdf_model;
        if (!urdf_model.initString(urdf_xml))
        {
          return;
        }
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
        {
          return;
        }
        KDL::Chain chain;
        if (!kdl_tree.getChain(base_frame, tool_frame, chain))
        {
          return;
        }
        for (size_t si = 1; si < chain.getNrOfSegments(); ++si)
        {
          const KDL::Joint &j = chain.getSegment(si).getJoint();
          if (j.getType() == KDL::Joint::None)
            continue;
          expected_joint_names_.push_back(j.getName());
        }
      }
      catch (...)
      {
      }
    }

  private:
    std::vector<std::string> expected_joint_names_;
  };

  // Helper to find and load URDF from file (supports both .urdf and .urdf.xacro)
  static std::string loadURDFFromFile()
  {
    // Explicit override via environment variable
    const char *override_path = std::getenv("TEST_URDF_PATH");
    if (override_path && std::strlen(override_path) > 0)
    {
      std::string path = override_path;
      bool is_xacro = path.size() > 6 && path.substr(path.size() - 6) == ".xacro";
      if (is_xacro)
      {
        std::string cmd = "xacro " + path;
        FILE *pipe = popen(cmd.c_str(), "r");
        if (pipe)
        {
          std::string result;
          char buffer[256];
          while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
            result += buffer;
          pclose(pipe);
          if (!result.empty())
            return result;
        }
      }
      else if (std::filesystem::exists(path))
      {
        std::ifstream file(path);
        if (file.is_open())
        {
          std::stringstream buf;
          buf << file.rdbuf();
          return buf.str();
        }
      }
    }
    // Get robot name from environment variable (defaults to gen3_2f85)
    const char *robot_name_env = std::getenv("TEST_ROBOT_NAME");
    std::string robot_name = robot_name_env ? std::string(robot_name_env) : "gen3_2f85";

    std::cout << "Looking for URDF for robot: " << robot_name << std::endl;

    // Try multiple strategies to find the URDF
    std::vector<std::pair<std::string, bool>> possible_paths; // {path, is_xacro}

    // Strategy 1: Check ROS_PACKAGE_PATH environment variable (both .urdf and .xacro)
    const char *ros_package_path = std::getenv("ROS_PACKAGE_PATH");
    if (ros_package_path)
    {
      std::string path_str(ros_package_path);
      size_t pos = 0;
      while ((pos = path_str.find(':', pos)) != std::string::npos)
      {
        std::string base = path_str.substr(0, pos);
        possible_paths.push_back(
            {base + "/ros2_kortex/kortex_description/robots/" + robot_name + ".urdf", false});
        possible_paths.push_back(
            {base + "/franka_description/robots/" + robot_name + "/" + robot_name + ".urdf.xacro",
             true});
        path_str.erase(0, pos + 1);
      }
      possible_paths.push_back(
          {path_str + "/ros2_kortex/kortex_description/robots/" + robot_name + ".urdf", false});
      possible_paths.push_back(
          {path_str + "/franka_description/robots/" + robot_name + "/" + robot_name + ".urdf.xacro",
           true});
    }

    // Strategy 2: Check AMENT_PREFIX_PATH (ROS 2 install path)
    const char *ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
    if (ament_prefix_path)
    {
      std::string path_str(ament_prefix_path);
      size_t pos = 0;
      while ((pos = path_str.find(':', pos)) != std::string::npos)
      {
        std::string base = path_str.substr(0, pos);
        possible_paths.push_back(
            {base + "/share/kortex_description/robots/" + robot_name + ".urdf", false});
        possible_paths.push_back({base + "/share/franka_description/robots/" + robot_name + "/" +
                                      robot_name + ".urdf.xacro",
                                  true});
        path_str.erase(0, pos + 1);
      }
      possible_paths.push_back(
          {path_str + "/share/kortex_description/robots/" + robot_name + ".urdf", false});
      possible_paths.push_back({path_str + "/share/franka_description/robots/" + robot_name + "/" +
                                    robot_name + ".urdf.xacro",
                                true});
    }

    // Strategy 3: Common workspace locations (both .urdf and .xacro)
    possible_paths.push_back(
        {"./src/ros2_kortex_ws/src/ros2_kortex/kortex_description/robots/" + robot_name + ".urdf",
         false});
    possible_paths.push_back(
        {"../kinova_ros2_ws/src/ros2_kortex_ws/src/ros2_kortex/kortex_description/robots/" +
             robot_name + ".urdf",
         false});
    possible_paths.push_back(
        {"../../kinova_ros2_ws/src/ros2_kortex_ws/src/ros2_kortex/kortex_description/robots/" +
             robot_name + ".urdf",
         false});
    possible_paths.push_back({"../franka_ros2_ws/src/franka_description/robots/" + robot_name +
                                  "/" + robot_name + ".urdf.xacro",
                              true});
    possible_paths.push_back({"../../franka_ros2_ws/src/franka_description/robots/" + robot_name +
                                  "/" + robot_name + ".urdf.xacro",
                              true});

    for (const auto &[path, is_xacro] : possible_paths)
    {
      if (std::filesystem::exists(path))
      {
        std::cout << "✓ Found " << (is_xacro ? "xacro file" : "URDF") << " at: " << path
                  << std::endl;

        if (is_xacro)
        {
          // Process xacro file using xacro command
          std::string cmd = "xacro " + path;
          FILE *pipe = popen(cmd.c_str(), "r");
          if (!pipe)
          {
            std::cerr << "Failed to execute xacro command" << std::endl;
            continue;
          }
          std::string result;
          char buffer[256];
          while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
          {
            result += buffer;
          }
          pclose(pipe);

          if (result.empty())
          {
            std::cerr << "xacro command returned empty result" << std::endl;
            continue;
          }
          return result;
        }
        else
        {
          // Load regular URDF file
          std::ifstream file(path);
          if (file.is_open())
          {
            std::stringstream buffer;
            buffer << file.rdbuf();
            return buffer.str();
          }
        }
      }
    }

    return "";
  }

  /// Generic FK unit test for any robot configuration
  /// Environment variables:
  ///   TEST_ROBOT_NAME: Robot URDF name (default: gen3_2f85)
  ///   TEST_BASE_FRAME: Base link frame name (default: gen3_base_link)
  ///   TEST_EE_FRAME: End effector frame name (default: gen3_end_effector_link)
  ///   TEST_NUM_JOINTS: Number of joints (default: 7)
  ///   TEST_JOINT_PREFIX: Prefix for joint names (default: gen3_joint_)
  ///
  /// Example for Kinova Gen3:
  ///   ./test_generic_fk_unit
  ///
  /* Example for Franka FR3:
    TEST_ROBOT_NAME=fr3
    TEST_BASE_FRAME=base
    TEST_EE_FRAME=fr3_hand_tcp
    TEST_NUM_JOINTS=7
    TEST_JOINT_PREFIX=fr3_joint
    ./test_generic_fk_unit
  */
  class GenericRobotFKTest : public ::testing::Test
  {
  protected:
    // Robot configuration (set from environment variables)
    static std::string robot_name;
    static std::string base_frame;
    static std::string ee_frame;
    static int num_joints;
    static std::string joint_prefix;

    static void SetUpTestSuite()
    {
      // Read robot configuration from environment variables
      const char *robot_name_env = std::getenv("TEST_ROBOT_NAME");
      robot_name = robot_name_env ? std::string(robot_name_env) : "gen3_2f85";

      const char *base_frame_env = std::getenv("TEST_BASE_FRAME");
      base_frame = base_frame_env ? std::string(base_frame_env) : "gen3_base_link";

      const char *ee_frame_env = std::getenv("TEST_EE_FRAME");
      ee_frame = ee_frame_env ? std::string(ee_frame_env) : "gen3_end_effector_link";

      const char *num_joints_env = std::getenv("TEST_NUM_JOINTS");
      num_joints = num_joints_env ? std::stoi(std::string(num_joints_env)) : 7;

      const char *joint_prefix_env = std::getenv("TEST_JOINT_PREFIX");
      joint_prefix = joint_prefix_env ? std::string(joint_prefix_env) : "gen3_joint_";

      std::cout << "\n=== Robot Configuration ===" << std::endl;
      std::cout << "Robot: " << robot_name << std::endl;
      std::cout << "Base frame: " << base_frame << std::endl;
      std::cout << "EE frame: " << ee_frame << std::endl;
      std::cout << "Joints: " << num_joints << " (prefix: " << joint_prefix << ")" << std::endl
                << std::endl;
    }

    void SetUp() override
    {
      // Initialize ROS 2 (only once)
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

      // Only create node once, skip setup for subsequent tests
      if (test_node_)
      {
        std::cout << "\n✓ FK Test Setup (reusing existing node)\n" << std::endl;
        return;
      }

      // Load URDF from file (no parameter server needed!)
      std::string urdf_xml = loadURDFFromFile();

      if (urdf_xml.empty())
      {
        GTEST_SKIP() << "Could not find URDF file at expected locations. "
                     << "Make sure kinova_ros2_ws is properly built.";
      }

      // Create test node
      test_node_ = std::make_shared<rclcpp::Node>("fk_test_node");
      // Node exists for logging/QoS consistency only (no subscriptions required)

      // Create the test component
      component_ = std::make_unique<TestFKComponent>();

      // Seed component with URDF and frame names per current API, and attach node interfaces
      component_->setRobotDescription(urdf_xml);
      component_->setFrameNames(base_frame, ee_frame);
      component_->setNodeForTesting(test_node_);
      // Ensure injected joint names match the KDL chain exactly
      component_->setExpectedJointNamesFromURDF(urdf_xml, base_frame, ee_frame);

      std::cout << "\n✓ FK Test Setup Complete - URDF loaded from file, ready to test\n"
                << std::endl;
    }

    void TearDown() override
    {
      // Don't destroy objects between tests - let them persist
      // This avoids cleanup crashes. They'll be destroyed at program exit.
    }

    void InjectJointState(const std::vector<double> &positions)
    {
      component_->injectJointState(positions, joint_prefix, num_joints);
    }

    bool PosesAreEqual(const CartesianPosition &p1, const CartesianPosition &p2,
                       double tolerance = 1e-4)
    {
      return (p1.translation - p2.translation).norm() < tolerance &&
             (p1.quaternion.coeffs() - p2.quaternion.coeffs()).norm() < tolerance;
    }

    static std::shared_ptr<rclcpp::Node> test_node_;
    static std::unique_ptr<TestFKComponent> component_;
  };

  // Static member initialization
  std::string GenericRobotFKTest::robot_name = "gen3_2f85";
  std::string GenericRobotFKTest::base_frame = "gen3_base_link";
  std::string GenericRobotFKTest::ee_frame = "gen3_end_effector_link";
  int GenericRobotFKTest::num_joints = 7;
  std::string GenericRobotFKTest::joint_prefix = "gen3_joint_";
  std::shared_ptr<rclcpp::Node> GenericRobotFKTest::test_node_;
  std::unique_ptr<TestFKComponent> GenericRobotFKTest::component_;

  // Test 1: FK computes zero pose for zero joint configuration
  TEST_F(GenericRobotFKTest, ZeroPoseForZeroJoints)
  {
    std::cout << "\n=== Test 1: Zero Pose for Zero Joints ===" << std::endl;
    std::vector<double> zero_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    InjectJointState(zero_config);

    CartesianPosition pose = component_->getCurrentEndEffectorPose();

    std::cout << "Pose at zero config:" << std::endl;
    std::cout << "  Position: [" << pose.translation.x() << ", " << pose.translation.y() << ", "
              << pose.translation.z() << "]" << std::endl;
    std::cout << "  Quaternion: [" << pose.quaternion.x() << ", " << pose.quaternion.y() << ", "
              << pose.quaternion.z() << ", " << pose.quaternion.w() << "]" << std::endl;

    // Verify quaternion is normalized
    double quat_norm = pose.quaternion.norm();
    EXPECT_NEAR(quat_norm, 1.0, 0.01) << "Quaternion should be normalized";

    // Verify it's a valid pose (not all zeros)
    EXPECT_FALSE(pose.translation.isZero(1e-6) && pose.quaternion.w() == 0.0)
        << "FK should compute a valid pose";

    // If Kinova Gen3 robot, verify expected zero pose numerics
    // Expected (xyzw quaternion order in logs):
    //   Translation: [-0.000, -0.025, 1.187]
    //   Quaternion (x,y,z,w): [0.000, -0.000, 0.000, 1.000]
    if (robot_name.rfind("gen3", 0) == 0)
    {
      const double pos_tol = 1e-3;  // 1 mm tolerance
      const double quat_tol = 1e-3; // small numeric tolerance
      EXPECT_NEAR(pose.translation.x(), 0.0, pos_tol);
      EXPECT_NEAR(pose.translation.y(), -0.025, pos_tol);
      EXPECT_NEAR(pose.translation.z(), 1.187, pos_tol);

      EXPECT_NEAR(pose.quaternion.x(), 0.0, quat_tol);
      EXPECT_NEAR(pose.quaternion.y(), 0.0, quat_tol);
      EXPECT_NEAR(pose.quaternion.z(), 0.0, quat_tol);
      EXPECT_NEAR(pose.quaternion.w(), 1.0, quat_tol);
    }

    std::cout << "✓ Test passed" << std::endl;
  }

  // Test 2: Different joint configs produce different poses
  TEST_F(GenericRobotFKTest, DifferentConfigsProduceDifferentPoses)
  {
    std::cout << "\n=== Test 2: Different Configs Produce Different Poses ===" << std::endl;

    // Config 1: Zero pose
    std::vector<double> config1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    InjectJointState(config1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow time for update
    CartesianPosition pose1 = component_->getCurrentEndEffectorPose();

    std::cout << "\nConfig 1 (all zeros):" << std::endl;
    std::cout << "  Position: [" << pose1.translation.x() << ", " << pose1.translation.y() << ", "
              << pose1.translation.z() << "]" << std::endl;
    std::cout << "  Quaternion: [" << pose1.quaternion.x() << ", " << pose1.quaternion.y() << ", "
              << pose1.quaternion.z() << ", " << pose1.quaternion.w() << "]" << std::endl;

    // Config 2: Joint 3 rotated 45 degrees (affects elbow position)
    std::vector<double> config2 = {0.0, 0.0, 0.7854, 0.0, 0.0, 0.0, 0.0}; // pi/4 on J3
    InjectJointState(config2);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow time for update
    CartesianPosition pose2 = component_->getCurrentEndEffectorPose();

    std::cout << "\nConfig 2 (J3=45°):" << std::endl;
    std::cout << "  Position: [" << pose2.translation.x() << ", " << pose2.translation.y() << ", "
              << pose2.translation.z() << "]" << std::endl;
    std::cout << "  Quaternion: [" << pose2.quaternion.x() << ", " << pose2.quaternion.y() << ", "
              << pose2.quaternion.z() << ", " << pose2.quaternion.w() << "]" << std::endl;

    // Poses should be different
    double position_diff = (pose1.translation - pose2.translation).norm();
    std::cout << "Position difference: " << position_diff << " m" << std::endl;

    EXPECT_GT(position_diff, 1e-4)
        << "FK should produce different poses for different joint configs (J1 rotation)";

    std::cout << "✓ Test passed" << std::endl;
  }

  // Test 3: Joint 2 rotation produces different pose
  TEST_F(GenericRobotFKTest, Joint2RotationChangesPose)
  {
    std::cout << "\n=== Test 3: Joint 2 Rotation Changes Pose ===" << std::endl;

    // Config 1: Zero
    std::vector<double> config1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    InjectJointState(config1);
    CartesianPosition pose1 = component_->getCurrentEndEffectorPose();

    // Config 2: Joint 2 rotated 45 degrees
    std::vector<double> config2 = {0.0, 0.7854, 0.0, 0.0, 0.0, 0.0, 0.0}; // pi/4
    InjectJointState(config2);
    CartesianPosition pose2 = component_->getCurrentEndEffectorPose();

    std::cout << "\nJ2 rotation test:" << std::endl;
    std::cout << "  Pose 1 (J2=0°): [" << pose1.translation.x() << ", " << pose1.translation.y()
              << ", " << pose1.translation.z() << "]" << std::endl;
    std::cout << "  Pose 2 (J2=45°): [" << pose2.translation.x() << ", " << pose2.translation.y()
              << ", " << pose2.translation.z() << "]" << std::endl;

    // Positions should be different
    double position_diff = (pose1.translation - pose2.translation).norm();
    std::cout << "Position difference: " << position_diff << " m" << std::endl;

    EXPECT_GT(position_diff, 1e-4) << "FK should produce different poses when J2 changes";

    std::cout << "✓ Test passed" << std::endl;
  }

  // Test 4: Multiple joint changes
  TEST_F(GenericRobotFKTest, MultipleJointRotations)
  {
    std::cout << "\n=== Test 4: Multiple Joint Rotations ===" << std::endl;

    std::vector<double> config = {0.5, 0.5, -0.3, 0.2, 0.4, -0.2, 0.1};
    InjectJointState(config);
    CartesianPosition pose = component_->getCurrentEndEffectorPose();

    std::cout << "\nMulti-joint config [0.5, 0.5, -0.3, 0.2, 0.4, -0.2, 0.1]:" << std::endl;
    std::cout << "  Position: [" << pose.translation.x() << ", " << pose.translation.y() << ", "
              << pose.translation.z() << "]" << std::endl;
    std::cout << "  Quaternion norm: " << pose.quaternion.norm() << std::endl;

    // Verify valid pose
    double quat_norm = pose.quaternion.norm();
    EXPECT_NEAR(quat_norm, 1.0, 0.01) << "Quaternion should be normalized";

    std::cout << "✓ Test passed" << std::endl;
  }

  // Test 5: FR3 home-ish configuration
  TEST_F(GenericRobotFKTest, FR3HomeishConfiguration)
  {
    std::cout << "\n=== Test 5: FR3 Home-ish Configuration ===" << std::endl;
    // Skip this FR3-specific test if the active robot is not FR3
    if (robot_name != "fr3")
    {
      GTEST_SKIP() << "Skipping FR3-specific test for robot '" << robot_name << "'";
    }
    // Positions: [0, -0.785398..., 0, -2.356194..., 0, 1.570796..., 0.785398...]
    std::vector<double> fr3_homeish = {
        0.0,
        -0.7853981633974483, // -pi/4
        0.0,
        -2.356194490192345, // -3pi/4
        0.0,
        1.5707963267948966, // pi/2
        0.7853981633974483  // pi/4
    };

    InjectJointState(fr3_homeish);
    CartesianPosition pose = component_->getCurrentEndEffectorPose();

    std::cout << "Pose for FR3 home-ish config:" << std::endl;
    std::cout << "  Position: [" << pose.translation.x() << ", " << pose.translation.y() << ", "
              << pose.translation.z() << "]" << std::endl;
    std::cout << "  Quaternion: [" << pose.quaternion.x() << ", " << pose.quaternion.y() << ", "
              << pose.quaternion.z() << ", " << pose.quaternion.w() << "]" << std::endl;

    // Basic validity checks
    double quat_norm = pose.quaternion.norm();
    EXPECT_NEAR(quat_norm, 1.0, 0.01) << "Quaternion should be normalized";
    EXPECT_FALSE(pose.translation.isZero(1e-6))
        << "Pose should not be at origin for home-ish config";

    std::cout << "✓ Test passed" << std::endl;
  }

} // namespace robot_interfaces

int main(int argc, char **argv)
{
  try
  {
    // Initialize GTest
    ::testing::InitGoogleTest(&argc, argv);

    // Run tests
    int result = RUN_ALL_TESTS();

    // Print summary
    std::cout << "\n========== Test Execution Complete ==========" << std::endl;
    std::cout << "Exit code: " << result << std::endl;

    // Exit immediately to avoid static cleanup issues
    // (using _Exit to skip destructors which may throw)
    _Exit(result);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Exception in main: " << e.what() << std::endl;
    _Exit(1);
  }
  catch (...)
  {
    std::cerr << "Unknown exception in main" << std::endl;
    _Exit(1);
  }
}
