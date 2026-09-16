#!/usr/bin/env python3
"""Move an RM75 to a launch-provided initial joint pose once."""

import time

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import Movej
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
READY_TIMEOUT_SECONDS = 30.0
RESULT_TIMEOUT_SECONDS = 120.0
ALREADY_AT_TARGET_TOLERANCE = 0.01
FEEDBACK_FRESHNESS_SECONDS = 0.5


class InitialPoseMover(Node):
    """Wait for real feedback, then send exactly one blocking MoveJ command."""

    def __init__(self):
        super().__init__("rm_75_initial_pose")
        self.declare_parameter("initial_joint_positions", [0.0] * 7)
        target = self.get_parameter("initial_joint_positions").value
        if len(target) != 7:
            raise ValueError("initial_joint_positions must contain seven values")

        self.target = [float(value) for value in target]
        self.current_positions = None
        self.last_feedback_at = None
        self.last_valid_feedback_at = None
        self.started_at = time.monotonic()
        self.command_sent_at = None
        self.finished = False
        self.failed = False

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
        if message.data:
            self.last_valid_feedback_at = time.monotonic()

    def _finish(self, *, failed=False):
        self.finished = True
        self.failed = failed

    def _result_callback(self, message):
        if self.command_sent_at is None or self.finished:
            return
        if message.data:
            self.get_logger().info("RM75 reached the configured initial pose")
            self._finish()
        else:
            self.get_logger().error("RM75 rejected or failed the initial-pose MoveJ")
            self._finish(failed=True)

    def _timer_callback(self):
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
        command.speed = 50
        command.block = True
        command.trajectory_connect = 0
        command.dof = 7
        self.command_sent_at = now
        self.command_publisher.publish(command)
        self.get_logger().warning(
            "Moving RM75 to the configured initial pose at 50% speed"
        )


def main(args=None):
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
