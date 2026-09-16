#!/usr/bin/env python3
"""等待真实关节反馈后，将 RM75 移动到启动参数指定的初始姿态。"""

import time

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import Movej
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# 按驱动命名顺序排列的七个关节名称。
JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
# 等待驱动订阅和有效反馈的最长时间。
READY_TIMEOUT_SECONDS = 30.0
# 等待阻塞式 MoveJ 返回结果的最长时间。
RESULT_TIMEOUT_SECONDS = 120.0
# 判定当前姿态已到达目标的最大关节误差，单位为弧度。
ALREADY_AT_TARGET_TOLERANCE = 0.01
# 判定关节状态和驱动反馈新鲜度的时间窗口。
FEEDBACK_FRESHNESS_SECONDS = 0.5


class InitialPoseMover(Node):
    """仅在收到新鲜有效反馈后发送一次阻塞式 MoveJ 指令。"""

    def __init__(self):
        """初始化目标关节角、状态订阅、指令发布器与超时定时器。"""
        super().__init__("rm_75_initial_pose")
        self.declare_parameter("initial_joint_positions", [0.0] * 7)
        target = self.get_parameter("initial_joint_positions").value
        if len(target) != 7:
            raise ValueError("initial_joint_positions must contain seven values")

        # 启动参数指定的七个目标关节角度。
        self.target = [float(value) for value in target]
        # 最近一次完整关节状态中的角度。
        self.current_positions = None
        # 最近一次完整关节状态的接收时间。
        self.last_feedback_at = None
        # 最近一次有效 UDP 反馈信号的接收时间。
        self.last_valid_feedback_at = None
        # 节点开始等待反馈的时间。
        self.started_at = time.monotonic()
        # 初始姿态指令发送时间，未发送时为空。
        self.command_sent_at = None
        # 节点动作是否结束。
        self.finished = False
        # 节点动作是否失败。
        self.failed = False

        # 向驱动发送一次初始姿态运动指令的发布器。
        self.command_publisher = self.create_publisher(
            Movej, "/rm_driver/movej_cmd", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.create_subscription(
            Bool,
            "/rm_driver/udp_feedback_valid",
            self._feedback_valid_callback,
            10,
        )
        self.create_subscription(
            Bool, "/rm_driver/movej_result", self._result_callback, 10
        )
        self.create_timer(0.1, self._timer_callback)
        self.get_logger().info(
            "Initial-pose mode enabled; waiting for RM75 feedback and driver"
        )

    def _joint_state_callback(self, message):
        """按关节名记录完整的关节状态与接收时间。"""
        if len(message.name) != len(message.position):
            return
        positions_by_name = dict(zip(message.name, message.position))
        if any(name not in positions_by_name for name in JOINT_NAMES):
            return
        self.current_positions = [
            float(positions_by_name[name]) for name in JOINT_NAMES
        ]
        self.last_feedback_at = time.monotonic()

    def _feedback_valid_callback(self, message):
        """记录驱动最新一次有效 UDP 反馈的接收时间。"""
        if message.data:
            self.last_valid_feedback_at = time.monotonic()

    def _finish(self, *, failed=False):
        """标记动作已结束及其失败状态。"""
        self.finished = True
        self.failed = failed

    def _result_callback(self, message):
        """处理驱动返回的 MoveJ 执行结果。"""
        if self.command_sent_at is None or self.finished:
            return
        if message.data:
            self.get_logger().info("RM75 reached the configured initial pose")
            self._finish()
        else:
            self.get_logger().error("RM75 rejected or failed the initial-pose MoveJ")
            self._finish(failed=True)

    def _timer_callback(self):
        """等待新鲜反馈和驱动订阅者，满足条件后发送目标或超时报错。"""
        if self.finished:
            return

        now = time.monotonic()
        if self.command_sent_at is not None:
            if now - self.command_sent_at > RESULT_TIMEOUT_SECONDS:
                self.get_logger().error("Timed out waiting for initial-pose MoveJ result")
                self._finish(failed=True)
            return

        if now - self.started_at > READY_TIMEOUT_SECONDS:
            self.get_logger().error(
                "Timed out waiting for /joint_states and /rm_driver/movej_cmd"
            )
            self._finish(failed=True)
            return

        if (
            self.current_positions is None
            or self.last_feedback_at is None
            or self.last_valid_feedback_at is None
            or now - self.last_feedback_at > FEEDBACK_FRESHNESS_SECONDS
            or now - self.last_valid_feedback_at > FEEDBACK_FRESHNESS_SECONDS
        ):
            return
        if self.command_publisher.get_subscription_count() == 0:
            return

        maximum_error = max(
            abs(current - target)
            for current, target in zip(self.current_positions, self.target)
        )
        if maximum_error <= ALREADY_AT_TARGET_TOLERANCE:
            self.get_logger().info("RM75 is already at the configured initial pose")
            self._finish()
            return

        command = Movej()
        command.joint = self.target
        command.speed = 20
        command.block = True
        command.trajectory_connect = 0
        command.dof = 7
        self.command_sent_at = now
        self.command_publisher.publish(command)
        self.get_logger().warning(
            "Moving RM75 to the configured initial pose at 20% speed"
        )


def main(args=None):
    """运行初始姿态节点，按动作结果返回进程退出码。"""
    rclpy.init(args=args)
    node = InitialPoseMover()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        failed = node.failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
