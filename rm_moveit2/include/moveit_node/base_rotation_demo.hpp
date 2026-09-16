/**
 * @file base_rotation_demo.hpp
 * @brief 生成并执行 RM75 底座扫转示教的关节目标。
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace moveit_node
{

/** @brief 第二关节的水平姿态目标，单位为弧度。 */
constexpr double kBaseRotationJoint2Target = 1.5707963267948966;
/** @brief 第一关节扫转的绝对角度边界，单位为弧度。 */
constexpr double kBaseRotationJoint1Limit = 3.1;
/** @brief 通用运动节点默认关闭底座扫转示教。 */
constexpr bool kDefaultBaseRotationDemoEnabled = false;

/** @brief 一步底座扫转示教及其完整关节目标。 */
struct BaseRotationStep
{
  /** @brief 用于日志与错误提示的步骤名称。 */
  std::string context;
  /** @brief 按活动关节顺序排列的目标角度。 */
  std::vector<double> joint_values;
};

/**
 * @brief 以当前关节状态为基础，生成先抬臂再向两侧扫转的目标。
 * @param joint_names 活动关节名称，顺序与当前关节值一致。
 * @param current_joint_values 当前关节角度。
 * @param[out] steps 生成的三个目标步骤；失败时为空。
 * @param[out] error 失败原因；成功时为空。
 * @return 输入有效且包含 joint1、joint2 时返回 true。
 */
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

  /** 第一关节在活动关节列表中的位置。 */
  const auto joint1 = std::find(joint_names.begin(), joint_names.end(), "joint1");
  /** 第二关节在活动关节列表中的位置。 */
  const auto joint2 = std::find(joint_names.begin(), joint_names.end(), "joint2");
  if (joint1 == joint_names.end() || joint2 == joint_names.end()) {
    error = "活动关节中必须同时包含 joint1 和 joint2";
    return false;
  }

  /** 第一关节目标在关节值数组中的下标。 */
  const auto joint1_index = static_cast<std::size_t>(joint1 - joint_names.begin());
  /** 第二关节目标在关节值数组中的下标。 */
  const auto joint2_index = static_cast<std::size_t>(joint2 - joint_names.begin());
  /** 保留其他关节当前角度的目标数组。 */
  auto target = current_joint_values;
  target[joint2_index] = kBaseRotationJoint2Target;
  steps.push_back({"base_rotation_joint2_horizontal", target});
  target[joint1_index] = -kBaseRotationJoint1Limit;
  steps.push_back({"base_rotation_joint1_negative_limit", target});
  target[joint1_index] = kBaseRotationJoint1Limit;
  steps.push_back({"base_rotation_joint1_positive_limit", target});
  return true;
}

/**
 * @brief 在启用时依次执行底座扫转目标，任一步失败即停止。
 * @param enabled 是否启用示教；关闭时不会调用执行器。
 * @param joint_names 活动关节名称。
 * @param current_joint_values 当前关节角度。
 * @param execute_step 执行单步关节目标的回调。
 * @param[out] error 目标生成或执行失败原因。
 * @return 全部执行成功或功能关闭时返回 true。
 */
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

  /** 待执行的底座扫转步骤。 */
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
