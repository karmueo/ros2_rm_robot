/**
 * @file linear_motion_node_parameters_test.cpp
 * @brief 验证通用 MoveIt 节点的底座扫转参数默认值。
 */
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "moveit_node/linear_motion_node.hpp"

/** @brief 通用节点没有显式传参时不执行底座扫转。 */
TEST(LinearMotionNodeParametersTest, BaseRotationDemoDefaultsToFalse)
{
  /** 测试进程不传递命令行参数。 */
  int argc = 0;
  /** ROS 初始化所需的空参数指针。 */
  char ** argv = nullptr;
  rclcpp::init(argc, argv);
  /** 使用默认选项构造的待测节点。 */
  auto node = std::make_shared<LinearMotionNode>(rclcpp::NodeOptions());

  EXPECT_FALSE(node->get_parameter("enable_base_rotation_demo").as_bool());

  node.reset();
  rclcpp::shutdown();
}
