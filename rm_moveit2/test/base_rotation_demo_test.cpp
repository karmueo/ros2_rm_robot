#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "moveit_node/base_rotation_demo.hpp"

namespace
{

TEST(BaseRotationDemoTest, BuildsInheritedTargetsByJointName)
{
  const std::vector<std::string> names{"joint3", "joint1", "joint2", "joint4"};
  const std::vector<double> current{0.4, 0.2, -0.7, 1.1};
  std::vector<moveit_node::BaseRotationStep> steps;
  std::string error;

  ASSERT_TRUE(moveit_node::buildBaseRotationDemoTargets(names, current, steps, error));
  ASSERT_EQ(steps.size(), 3u);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[0], 0.4);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[1], 0.2);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[2], 1.5707963267948966);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[3], 1.1);
  EXPECT_DOUBLE_EQ(steps[1].joint_values[1], -3.1);
  EXPECT_DOUBLE_EQ(steps[1].joint_values[2], 1.5707963267948966);
  EXPECT_DOUBLE_EQ(steps[2].joint_values[1], 3.1);
  EXPECT_DOUBLE_EQ(steps[2].joint_values[2], 1.5707963267948966);
  EXPECT_DOUBLE_EQ(steps[2].joint_values[0], steps[1].joint_values[0]);
  EXPECT_DOUBLE_EQ(steps[2].joint_values[3], steps[1].joint_values[3]);
}

TEST(BaseRotationDemoTest, RejectsMissingRequiredJoint)
{
  std::vector<moveit_node::BaseRotationStep> steps;
  std::string error;

  EXPECT_FALSE(moveit_node::buildBaseRotationDemoTargets(
      {"joint1", "joint3"}, {0.0, 0.0}, steps, error));
  EXPECT_TRUE(steps.empty());
  EXPECT_NE(error.find("joint2"), std::string::npos);
}

TEST(BaseRotationDemoTest, RejectsMismatchedJointStateSize)
{
  std::vector<moveit_node::BaseRotationStep> steps;
  std::string error;

  EXPECT_FALSE(moveit_node::buildBaseRotationDemoTargets(
      {"joint1", "joint2"}, {0.0}, steps, error));
  EXPECT_TRUE(steps.empty());
}

TEST(BaseRotationDemoTest, BaseRotationIsDisabledByGenericDefault)
{
  EXPECT_FALSE(moveit_node::kDefaultBaseRotationDemoEnabled);
}

TEST(BaseRotationDemoTest, DisabledDemoDoesNotExecuteAnyStep)
{
  int call_count = 0;
  std::string error;

  ASSERT_TRUE(moveit_node::executeBaseRotationDemo(
      false,
      {"joint1", "joint2"},
      {0.0, 0.0},
      [&call_count](const moveit_node::BaseRotationStep &) {
        ++call_count;
        return true;
      },
      error));
  EXPECT_EQ(call_count, 0);
}

TEST(BaseRotationDemoTest, StopsAfterFirstFailedExecution)
{
  std::vector<std::string> contexts;
  std::string error;

  EXPECT_FALSE(moveit_node::executeBaseRotationDemo(
      true,
      {"joint1", "joint2"},
      {0.0, 0.0},
      [&contexts](const moveit_node::BaseRotationStep & step) {
        contexts.push_back(step.context);
        return contexts.size() == 1u;
      },
      error));
  ASSERT_EQ(contexts.size(), 2u);
  EXPECT_EQ(contexts[0], "base_rotation_joint2_horizontal");
  EXPECT_EQ(contexts[1], "base_rotation_joint1_negative_limit");
  EXPECT_NE(error.find("base_rotation_joint1_negative_limit"), std::string::npos);
}

}  // namespace
