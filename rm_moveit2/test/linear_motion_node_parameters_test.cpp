#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "moveit_node/linear_motion_node.hpp"

TEST(LinearMotionNodeParametersTest, BaseRotationDemoDefaultsToFalse)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LinearMotionNode>(rclcpp::NodeOptions());

  EXPECT_FALSE(node->get_parameter("enable_base_rotation_demo").as_bool());

  node.reset();
  rclcpp::shutdown();
}
