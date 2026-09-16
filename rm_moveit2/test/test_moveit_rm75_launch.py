"""检查 RM75 MoveIt 启动文件中底座扫转参数的默认值及传递方式。"""

import ast
from pathlib import Path
import unittest


# 待检查的 RM75 MoveIt 启动文件路径。
LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "moveit_rm75.launch.py"


def call_name(node):
    """从 AST 调用节点提取可用于测试的函数名。"""
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return None


class Rm75LaunchTest(unittest.TestCase):
    """验证 RM75 专用启动文件的底座扫转参数配置。"""

    @classmethod
    def setUpClass(cls):
        """解析待测启动文件，供各测试复用。"""
        cls.tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"), str(LAUNCH_FILE))

    def test_declares_base_rotation_demo_enabled_by_default(self):
        """确认 RM75 启动参数默认启用底座扫转。"""
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
        """确认启动参数以布尔类型传入节点。"""
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
