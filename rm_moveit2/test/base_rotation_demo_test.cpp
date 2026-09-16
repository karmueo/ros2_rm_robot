/**
 * @file base_rotation_demo_test.cpp
 * @brief 验证底座扫转目标生成、缺失关节处理及执行失败时的停止行为。
 */
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "moveit_node/base_rotation_demo.hpp"

namespace
{

/** @brief 验证按关节名称定位目标，其余关节继承当前角度。 */
TEST(BaseRotationDemoTest, BuildsInheritedTargetsByJointName)
{
  /** 与关节值顺序对应的活动关节名称。 */
  const std::vector<std::string> names{"joint3", "joint1", "joint2", "joint4"};
  /** 测试使用的当前关节角度。 */
  const std::vector<double> current{0.4, 0.2, -0.7, 1.1};
  /** 待检查的生成目标步骤。 */
  std::vector<moveit_node::BaseRotationStep> steps;
  /** 目标生成或执行失败原因。 */
  std::string error;

  ASSERT_TRUE(moveit_node::buildBaseRotationDemoTargets(names, current, steps, error));
  ASSERT_EQ(steps.size(), 3u);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[0], 0.4);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[1], 0.2);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[2], 1.5707963267948966);
  EXPECT_DOUBLE_EQ(steps[0].joint_values[3], 1.1);

  EXPECT_DOUBLE_EQ(steps[1].joint_values[1], -3.1);
  EXPECT_DOUBLE_EQ(steps[1].joint_values[0], 0.4);
  EXPECT_DOUBLE_EQ(steps[1].joint_values[2], 1.5707963267948966);
  EXPECT_DOUBLE_EQ(steps[1].joint_values[3], 1.1);

  EXPECT_DOUBLE_EQ(steps[2].joint_values[1], 3.1);
  EXPECT_DOUBLE_EQ(steps[2].joint_values[0], 0.4);
  EXPECT_DOUBLE_EQ(steps[2].joint_values[2], 1.5707963267948966);
  EXPECT_DOUBLE_EQ(steps[2].joint_values[3], 1.1);
}

/** @brief 缺少第二关节时拒绝生成扫转目标。 */
TEST(BaseRotationDemoTest, RejectsMissingRequiredJoint)
{
  /** 待检查的生成目标步骤。 */
  std::vector<moveit_node::BaseRotationStep> steps;
  /** 目标生成或执行失败原因。 */
  std::string error;

  EXPECT_FALSE(moveit_node::buildBaseRotationDemoTargets(
      {"joint1", "joint3"}, {0.0, 0.0}, steps, error));
  EXPECT_TRUE(steps.empty());
  EXPECT_NE(error.find("joint2"), std::string::npos);
}

/** @brief 缺少第一关节时拒绝生成扫转目标。 */
TEST(BaseRotationDemoTest, RejectsMissingBaseRotationJoint1)
{
  /** 待检查的生成目标步骤。 */
  std::vector<moveit_node::BaseRotationStep> steps;
  /** 目标生成或执行失败原因。 */
  std::string error;

  EXPECT_FALSE(moveit_node::buildBaseRotationDemoTargets(
      {"joint2", "joint3"}, {0.0, 0.0}, steps, error));
  EXPECT_TRUE(steps.empty());
  EXPECT_NE(error.find("joint1"), std::string::npos);
}

/** @brief 拒绝关节值少于活动关节名称的状态。 */
TEST(BaseRotationDemoTest, RejectsMismatchedJointStateSize)
{
  /** 待检查的生成目标步骤。 */
  std::vector<moveit_node::BaseRotationStep> steps;
  /** 目标生成或执行失败原因。 */
  std::string error;

  EXPECT_FALSE(moveit_node::buildBaseRotationDemoTargets(
      {"joint1", "joint2"}, {0.0}, steps, error));
  EXPECT_TRUE(steps.empty());
  EXPECT_FALSE(error.empty());
}

/** @brief 拒绝关节值多于活动关节名称的状态。 */
TEST(BaseRotationDemoTest, RejectsOversizedCurrentStateVector)
{
  /** 待检查的生成目标步骤。 */
  std::vector<moveit_node::BaseRotationStep> steps;
  /** 目标生成或执行失败原因。 */
  std::string error;

  EXPECT_FALSE(moveit_node::buildBaseRotationDemoTargets(
      {"joint1", "joint2"}, {0.0, 0.0, 0.0}, steps, error));
  EXPECT_TRUE(steps.empty());
  EXPECT_FALSE(error.empty());
}

/** @brief 通用运动节点默认关闭底座扫转。 */
TEST(BaseRotationDemoTest, BaseRotationIsDisabledByGenericDefault)
{
  EXPECT_FALSE(moveit_node::kDefaultBaseRotationDemoEnabled);
}

/** @brief 功能关闭时不调用执行回调。 */
TEST(BaseRotationDemoTest, DisabledDemoDoesNotExecuteAnyStep)
{
  /** 执行回调被调用的次数。 */
  int call_count = 0;
  /** 目标生成或执行失败原因。 */
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

/** @brief 任一步执行失败时不再执行后续动作。 */
TEST(BaseRotationDemoTest, StopsAfterFirstFailedExecution)
{
  /** 已执行步骤的名称。 */
  std::vector<std::string> contexts;
  /** 目标生成或执行失败原因。 */
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
