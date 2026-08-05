#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace moveit_node
{

constexpr double kBaseRotationJoint2Target = 1.5707963267948966;
constexpr double kBaseRotationJoint1Limit = 3.1;
constexpr bool kDefaultBaseRotationDemoEnabled = false;

struct BaseRotationStep
{
  std::string context;
  std::vector<double> joint_values;
};

inline bool buildBaseRotationDemoTargets(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & current_joint_values,
  std::vector<BaseRotationStep> & steps,
  std::string & error)
{
  steps.clear();
  error.clear();
  if (joint_names.size() != current_joint_values.size()) {
    error = "活动关节名称与当前关节值数量不一致";
    return false;
  }

  const auto joint1 = std::find(joint_names.begin(), joint_names.end(), "joint1");
  const auto joint2 = std::find(joint_names.begin(), joint_names.end(), "joint2");
  if (joint1 == joint_names.end() || joint2 == joint_names.end()) {
    error = "活动关节中必须同时包含 joint1 和 joint2";
    return false;
  }

  const auto joint1_index = static_cast<std::size_t>(joint1 - joint_names.begin());
  const auto joint2_index = static_cast<std::size_t>(joint2 - joint_names.begin());
  auto target = current_joint_values;
  target[joint2_index] = kBaseRotationJoint2Target;
  steps.push_back({"base_rotation_joint2_horizontal", target});
  target[joint1_index] = -kBaseRotationJoint1Limit;
  steps.push_back({"base_rotation_joint1_negative_limit", target});
  target[joint1_index] = kBaseRotationJoint1Limit;
  steps.push_back({"base_rotation_joint1_positive_limit", target});
  return true;
}

inline bool executeBaseRotationDemo(
  bool enabled,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & current_joint_values,
  const std::function<bool(const BaseRotationStep &)> & execute_step,
  std::string & error)
{
  error.clear();
  if (!enabled) {
    return true;
  }
  if (!execute_step) {
    error = "底座扫转示教缺少执行器";
    return false;
  }

  std::vector<BaseRotationStep> steps;
  if (!buildBaseRotationDemoTargets(
        joint_names, current_joint_values, steps, error)) {
    return false;
  }
  for (const auto & step : steps) {
    if (!execute_step(step)) {
      error = "底座扫转示教步骤执行失败: " + step.context;
      return false;
    }
  }
  return true;
}

}  // namespace moveit_node
