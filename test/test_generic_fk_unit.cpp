#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "robot_interfaces/generic_component.hpp"

namespace robot_interfaces
{
  // Concrete class for testing the protected logic of GenericComponent
  class TestComponent : public GenericComponent
  {
  public:
    using GenericComponent::GenericComponent;

    bool setCommand(const CommandVariant & /*command*/) override
    {
      return true;
    }

    void set_test_state_names(const std::vector<std::string>& names) {
        this->state_names = names;
    }
  };

  class PinocchioFKTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      urdf_xml_ = R"(
        <?xml version="1.0"?>
        <robot name="test_robot">
          <link name="base_link"/>
          <link name="link1">
            <inertial>
              <mass value="1.0"/>
              <origin xyz="0 0 0.5"/>
              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
          </link>
          <link name="tool0">
            <inertial>
              <mass value="1.0"/>
              <origin xyz="0 0.5 0"/>
              <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
          </link>

          <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="link1"/>
            <axis xyz="0 0 1"/>
            <origin xyz="0 0 1"/>
            <limit effort="100.0" lower="-3.14" upper="3.14" velocity="1.0"/>
          </joint>

          <joint name="joint2" type="revolute">
            <parent link="link1"/>
            <child link="tool0"/>
            <axis xyz="0 0 1"/>
            <origin xyz="0 1 0"/>
            <limit effort="100.0" lower="-3.14" upper="3.14" velocity="1.0"/>
          </joint>
        </robot>)";

      state_interface_names_ = {"joint1/position", "joint1/velocity", "joint1/effort",
                                "joint2/position", "joint2/velocity", "joint2/effort"};

      component_ = std::make_unique<TestComponent>("test_robot", state_interface_names_.size(), 0);

      component_->set_test_state_names(state_interface_names_);

      ASSERT_TRUE(component_->initKinematics(urdf_xml_, "tool0"));

      pos_values_ = {0.0, 0.0}; // Values for joint1, joint2
      vel_values_ = {0.0, 0.0};
      eff_values_ = {0.0, 0.0};

      std::vector<hardware_interface::StateInterface> state_ifs;
      state_ifs.emplace_back("joint1", "position", &pos_values_[0]);
      state_ifs.emplace_back("joint1", "velocity", &vel_values_[0]);
      state_ifs.emplace_back("joint1", "effort", &eff_values_[0]);
      state_ifs.emplace_back("joint2", "position", &pos_values_[1]);
      state_ifs.emplace_back("joint2", "velocity", &vel_values_[1]);
      state_ifs.emplace_back("joint2", "effort", &eff_values_[1]);

      for (auto &sif : state_ifs)
      {
        loaned_state_ifs_.emplace_back(sif);
      }

      ASSERT_TRUE(component_->assign_loaned_state(loaned_state_ifs_));
    }

    std::string urdf_xml_;
    std::vector<std::string> state_interface_names_;
    std::unique_ptr<TestComponent> component_;

    std::vector<double> pos_values_;
    std::vector<double> vel_values_;
    std::vector<double> eff_values_;
    std::vector<hardware_interface::LoanedStateInterface> loaned_state_ifs_;
  };

  TEST_F(PinocchioFKTest, TestForwardKinematics)
  {
    // Move J1 by 90 degrees (PI/2)
    pos_values_[0] = 1.57079632679;
    pos_values_[1] = 0.0;

    component_->syncState();
    CartesianPosition pose = component_->getCurrentEndEffectorPose();

    EXPECT_NEAR(pose.translation.x(), -1.0, 1e-3);
    EXPECT_NEAR(pose.translation.y(), 0.0, 1e-3);
    EXPECT_NEAR(pose.translation.z(), 1.0, 1e-3);
  }

  TEST_F(PinocchioFKTest, TestJacobianSize)
  {
    component_->syncState();
    Eigen::MatrixXd J = component_->getEndEffectorJacobian();

    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), 2);
  }

  TEST_F(PinocchioFKTest, TestMassMatrixSymmetry)
  {
    pos_values_[0] = 0.5;
    pos_values_[1] = -0.2;
    component_->syncState();

    Eigen::MatrixXd M = component_->getMassMatrix();

    EXPECT_EQ(M.rows(), 2);
    EXPECT_EQ(M.cols(), 2);
    EXPECT_TRUE(M.isApprox(M.transpose()));
    EXPECT_GT(M(0, 0), 0.0);
    EXPECT_GT(M(1, 1), 0.0);
  }

  TEST_F(PinocchioFKTest, TestNonLinearEffects)
  {
    pos_values_[0] = 0.7;
    vel_values_[0] = 1.0; 
    component_->syncState();

    Eigen::VectorXd nle = component_->getNonLinearEffects();
    EXPECT_EQ(nle.size(), 2);
    EXPECT_TRUE(nle.norm() >= 0.0);
  }

} // namespace robot_interfaces

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}