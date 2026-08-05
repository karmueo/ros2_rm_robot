# RM75 底座扫转示教 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 RM75 MoveIt 启动流程中增加默认启用、受真实关节限位约束的 `joint2=π/2`、`joint1=-3.1` 到 `joint1=+3.1` 底座扫转示教，同时保持其他启动入口和关闭该参数时的原有行为。

**Architecture:** 将“按活动关节名称从当前值生成三组继承式目标”和“启用/失败短路/按序执行”的逻辑放入不依赖 ROS/MoveIt 的 `base_rotation_demo.hpp`，通过回调复用现有 MoveIt 动作并便于直接单元测试。`simple_moveit_node.cpp` 只读取参数、提供 `moveToJointTarget` 回调并处理最终错误；`LinearMotionNode` 只声明兼容性默认参数并暴露活动关节名称。RM75 launch 文件显式传入布尔值，其他型号不增加参数。

**Tech Stack:** C++14、ROS 2 Humble、MoveIt 2、ament_cmake、ament_cmake_gtest、ament_cmake_pytest、Python `ast`。

## Global Constraints

- `enable_base_rotation_demo` 在 RM75 launch 中默认值必须是字符串 `"true"`，并通过 `ParameterValue(..., value_type=bool)` 传给节点。
- `LinearMotionNode` 的通用默认值必须是 `false`。
- 目标顺序必须是 `joint2=π/2`、`joint1=-3.1`、`joint1=+3.1`。
- 只按活动关节名称定位 `joint1` 和 `joint2`；缺少任一关节或当前值数量不一致时拒绝执行。
- 每个目标从前一个目标继承，非目标关节保持最初当前值。
- 不修改 URDF、SRDF、控制器或 MoveIt 关节限位。
- 任何扫转步骤失败都必须停止节点，不得进入启动姿态、笛卡尔示教或回零流程。
- 保留工作区中与本功能无关的已有 `.gitignore`、`rm_moveit2_config/rm_65_config/package.xml` 和 `rm_driver/lib/libapi_cpp.so` 改动。

---

### Task 1: 为目标生成器和参数默认值建立失败测试

**Files:**
- Create: `rm_moveit2/test/base_rotation_demo_test.cpp`
- Create: `rm_moveit2/test/linear_motion_node_parameters_test.cpp`
- Create: `rm_moveit2/test/test_moveit_rm75_launch.py`
- Modify: `rm_moveit2/CMakeLists.txt`
- Modify: `rm_moveit2/package.xml`

**Interfaces:**
- Tests consume the not-yet-created `moveit_node::buildBaseRotationDemoTargets`, `moveit_node::executeBaseRotationDemo`, `moveit_node::BaseRotationStep`, and `moveit_node::kDefaultBaseRotationDemoEnabled`.
- The node-parameter test compiles against `LinearMotionNode` and expects `get_parameter("enable_base_rotation_demo").as_bool()` to be `false` without overrides.
- The launch test reads `rm_moveit2/launch/moveit_rm75.launch.py` with `ast`, so it does not require `rm_75_config` to be installed just to inspect launch declarations.

- [ ] **Step 1: Add the failing C++ target-generation tests**

```cpp
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
```

- [ ] **Step 2: Add the failing node default-parameter test**

```cpp
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
```

- [ ] **Step 3: Add the failing RM75 launch source test**

```python
import ast
from pathlib import Path
import unittest


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "moveit_rm75.launch.py"


def call_name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return None


class Rm75LaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"), str(LAUNCH_FILE))

    def test_declares_base_rotation_demo_enabled_by_default(self):
        declarations = [
            node for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and call_name(node.func) == "DeclareLaunchArgument"
        ]
        matches = [
            node for node in declarations
            if node.args and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "enable_base_rotation_demo"
        ]
        self.assertEqual(len(matches), 1)
        default = next(keyword.value for keyword in matches[0].keywords
                       if keyword.arg == "default_value")
        self.assertIsInstance(default, ast.Constant)
        self.assertEqual(default.value, "true")

    def test_passes_base_rotation_demo_as_boolean_parameter(self):
        entries = [
            value for node in ast.walk(self.tree) if isinstance(node, ast.Dict)
            for key, value in zip(node.keys, node.values)
            if isinstance(key, ast.Constant) and key.value == "enable_base_rotation_demo"
        ]
        self.assertEqual(len(entries), 1)
        self.assertEqual(call_name(entries[0].func), "ParameterValue")
        keywords = {keyword.arg: keyword.value for keyword in entries[0].keywords}
        self.assertEqual(call_name(keywords["value"].func), "LaunchConfiguration")
        self.assertIsInstance(keywords["value_type"], ast.Name)
        self.assertEqual(keywords["value_type"].id, "bool")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 4: Register tests and test dependencies without adding production code**

Add under `if(BUILD_TESTING)` in `rm_moveit2/CMakeLists.txt`:

```cmake
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(test_base_rotation_demo test/base_rotation_demo_test.cpp)
target_include_directories(test_base_rotation_demo PRIVATE include)

ament_add_gtest(test_linear_motion_node_parameters
  test/linear_motion_node_parameters_test.cpp
  src/linear_motion_node.cpp
)
target_include_directories(test_linear_motion_node_parameters PRIVATE include)
ament_target_dependencies(
  test_linear_motion_node_parameters
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
)

ament_add_pytest_test(test_moveit_rm75_launch test/test_moveit_rm75_launch.py)
```

Add `ament_cmake_gtest` and `ament_cmake_pytest` as `<test_depend>` entries in `rm_moveit2/package.xml`.

- [ ] **Step 5: Run only the new tests and verify RED**

Run from `/home/scl/work/ros2_ws`:

```bash
colcon test --packages-select rm_moveit2 --ctest-args -R 'test_base_rotation_demo|test_linear_motion_node_parameters|test_moveit_rm75_launch' --event-handlers console_direct+
```

Expected: the launch test fails because the declaration is absent, the target test fails because `moveit_node/base_rotation_demo.hpp` is absent, and the node test fails because the parameter is absent. Fix test setup errors until the failures are caused by the missing feature, then continue without changing the tests to pass prematurely.

- [ ] **Step 6: Commit the test-first scaffolding**

```bash
git add rm_moveit2/test rm_moveit2/CMakeLists.txt rm_moveit2/package.xml
git commit -m "test(rm_moveit2): add RM75 base rotation coverage"
```

### Task 2: Implement the pure target generator and node compatibility surface

**Files:**
- Create: `rm_moveit2/include/moveit_node/base_rotation_demo.hpp`
- Modify: `rm_moveit2/include/moveit_node/linear_motion_node.hpp`
- Modify: `rm_moveit2/src/linear_motion_node.cpp`

**Interfaces:**
- Produce `moveit_node::BaseRotationStep { std::string context; std::vector<double> joint_values; }`.
- Produce `moveit_node::buildBaseRotationDemoTargets(const std::vector<std::string>&, const std::vector<double>&, std::vector<BaseRotationStep>&, std::string&) -> bool`.
- Produce `moveit_node::kDefaultBaseRotationDemoEnabled == false`.
- Produce `LinearMotionNode::getActiveJointNames() const -> const std::vector<std::string>&` for name-based target generation.

- [ ] **Step 1: Add the minimal name-based generator and execution wrapper to make the C++ tests pass**

Create `base_rotation_demo.hpp` with the following behavior:

```cpp
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
```

- [ ] **Step 2: Declare and implement activity-name access**

Add this documented public method after `getJointCount()` in the header:

```cpp
/**
 * @brief 获取当前规划组的活动关节名称，顺序与关节目标数组一致。
 *
 * @return 当前活动关节名称列表。
 */
const std::vector<std::string> & getActiveJointNames() const;
```

Implement immediately after `LinearMotionNode::getJointCount()`:

```cpp
const std::vector<std::string> & LinearMotionNode::getActiveJointNames() const
{
  return joint_names_;
}
```

Include `moveit_node/base_rotation_demo.hpp` in `linear_motion_node.cpp` and use `moveit_node::kDefaultBaseRotationDemoEnabled` in the constructor declaration added in Task 3.

- [ ] **Step 3: Run the pure and node tests to verify GREEN for this task**

Run:

```bash
colcon test --packages-select rm_moveit2 --ctest-args -R 'test_base_rotation_demo|test_linear_motion_node_parameters' --event-handlers console_direct+
```

Expected: all target-generation assertions and the default-parameter assertion pass; launch assertions remain red until Task 4.

- [ ] **Step 4: Commit the generator and compatibility surface**

```bash
git add rm_moveit2/include/moveit_node/base_rotation_demo.hpp rm_moveit2/include/moveit_node/linear_motion_node.hpp rm_moveit2/src/linear_motion_node.cpp
git commit -m "feat(rm_moveit2): add name-based base rotation targets"
```

### Task 3: Integrate the guarded sweep into the existing MoveIt flow

**Files:**
- Modify: `rm_moveit2/src/linear_motion_node.cpp`
- Modify: `rm_moveit2/src/simple_moveit_node.cpp`

**Interfaces:**
- Consume `moveit_node::executeBaseRotationDemo`, `LinearMotionNode::getActiveJointNames()`, and `LinearMotionNode::moveToJointTarget`.
- Produce the ordered contexts `base_rotation_joint2_horizontal`, `base_rotation_joint1_negative_limit`, and `base_rotation_joint1_positive_limit` through the existing motion API.

- [ ] **Step 1: Declare the compatibility parameter with a false default**

In `LinearMotionNode::LinearMotionNode`, add alongside the other demo parameters:

```cpp
declare_parameter_if_needed(
  "enable_base_rotation_demo", moveit_node::kDefaultBaseRotationDemoEnabled);
```

- [ ] **Step 2: Implement the guarded three-step execution in `main`**

Include the helper header. After the existing `enable_pose_target` early-return branch and before `startup_joint_target` selection, add this logic:

```cpp
const bool enable_base_rotation_demo =
  node->get_parameter("enable_base_rotation_demo").as_bool();

std::string rotation_error;
const auto execute_rotation_step = [&node](const moveit_node::BaseRotationStep & step) {
    RCLCPP_INFO(node->get_logger(), "=== 底座扫转示教: %s ===", step.context.c_str());
    return node->moveToJointTarget(step.joint_values, step.context);
};

if (!moveit_node::executeBaseRotationDemo(
      enable_base_rotation_demo,
      node->getActiveJointNames(),
      joint_status,
      execute_rotation_step,
      rotation_error)) {
  RCLCPP_ERROR(
    node->get_logger(), "底座扫转示教失败: %s，停止后续示教", rotation_error.c_str());
  rclcpp::shutdown();
  spinner.join();
  return 1;
}
```

The existing startup target, Cartesian demo, and return-to-zero code must remain after this block. When the parameter is false, the block must not call the helper or `moveToJointTarget`, so the current sequence is unchanged.

- [ ] **Step 3: Build and run all package tests to verify GREEN**

Run:

```bash
colcon test --packages-select rm_moveit2 --event-handlers console_direct+
```

Expected: the package tests pass, including the default-false test; the launch test may still be red until Task 4 if the launch argument has not been added.

- [ ] **Step 4: Commit the node integration**

```bash
git add rm_moveit2/src/simple_moveit_node.cpp rm_moveit2/src/linear_motion_node.cpp
git commit -m "feat(rm_moveit2): run guarded RM75 base rotation demo"
```

### Task 4: Add the RM75-only launch switch and finish launch coverage

**Files:**
- Modify: `rm_moveit2/launch/moveit_rm75.launch.py`
- Test: `rm_moveit2/test/test_moveit_rm75_launch.py`

**Interfaces:**
- Consume the node parameter `enable_base_rotation_demo` from Task 3.
- Produce the RM75-only launch argument with default `"true"` and a boolean `ParameterValue` passed to `simple_moveit_node`.

- [ ] **Step 1: Declare the RM75 launch argument**

Add with the other `DeclareLaunchArgument` entries:

```python
DeclareLaunchArgument("enable_base_rotation_demo", default_value="true"),
```

- [ ] **Step 2: Pass the launch configuration as a boolean node parameter**

Add to the node parameter dictionary:

```python
"enable_base_rotation_demo": ParameterValue(
    LaunchConfiguration("enable_base_rotation_demo"), value_type=bool
),
```

Do not add this argument to `moveit_eco63.launch.py` or `moveit_rx75.launch.py`.

- [ ] **Step 3: Run the launch test and syntax checks**

Run:

```bash
colcon test --packages-select rm_moveit2 --ctest-args -R 'test_moveit_rm75_launch' --event-handlers console_direct+
python3 -m py_compile rm_moveit2/launch/moveit_rm75.launch.py rm_moveit2/launch/moveit_eco63.launch.py rm_moveit2/launch/moveit_rx75.launch.py
```

Expected: the launch test reports two passing tests and Python compilation exits with status 0. Remove any generated `__pycache__` files before reviewing the diff.

- [ ] **Step 4: Commit the RM75 launch switch**

```bash
git add rm_moveit2/launch/moveit_rm75.launch.py
git commit -m "feat(rm_moveit2): enable RM75 base rotation demo by launch"
```

### Task 5: Full verification and handoff

**Files:**
- Verify: `rm_moveit2` package sources, tests, launch files, and the implementation plan.

- [ ] **Step 1: Re-read the spec and check each requirement against the diff**

Confirm explicitly: RM75 default true; node default false; exact three target values and order; name-based lookup; unchanged non-target joints; missing-joint failure; failure short-circuit; other launch entries unchanged; no URDF/SRDF/controller edits.

- [ ] **Step 2: Run the complete package test command**

```bash
colcon test --packages-select rm_moveit2 --event-handlers console_direct+
```

Expected: exit code 0 and all registered tests pass with zero failures.

- [ ] **Step 3: Build the requested package**

```bash
colcon build --packages-select rm_moveit2 --event-handlers console_direct+
```

Expected: `rm_moveit2` finishes successfully. If dependency discovery fails because the workspace is not sourced with the required MoveIt/config packages, report that exact failure and retain the successful unit/launch checks.

- [ ] **Step 4: Check the final worktree without touching unrelated changes**

```bash
git status --short
git diff --check
git diff --stat HEAD
```

Expected: only the intended RM75 implementation, tests, and plan commits are attributed to this work; the pre-existing unrelated changes remain untouched. Report test/build command results and any environment-limited verification separately.
