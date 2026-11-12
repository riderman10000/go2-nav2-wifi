#!/usr/bin/env python3
"""Sport bridge on robot (internal DDS): /cmd_vel -> /api/sport/request.

Wi-Fi relay forwards /cmd_vel from laptop; this node must run on the robot.
Env: GO2_MAX_LINEAR, GO2_MAX_ANGULAR (optional caps; unset = no clamp),
GO2_CMD_TIMEOUT, GO2_SPORT_WALK_MODE (free|classic).
"""

from __future__ import annotations

import json
import signal
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from unitree_api.msg import Request, Response  # go2-agri: recovery/avoid

API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_RECOVERY_STAND = 1006
API_MOVE = 1008
API_SWITCH_JOYSTICK = 1027
API_FREE_WALK = 2045
API_CLASSIC_WALK = 2049

TOPIC_REQUEST = "/api/sport/request"
API_AUTORECOVERY_SET = 2054  # go2-agri: recovery/avoid
# Firmware obstacle avoidance (unitree_sdk2 obstacles_avoid_api).  # go2-agri: recovery/avoid
AVOID_SWITCH_SET, AVOID_SWITCH_GET, AVOID_MOVE, AVOID_USE_API = 1001, 1002, 1003, 1004  # go2-agri: recovery/avoid
TOPIC_AVOID = "/api/obstacles_avoid/request"  # go2-agri: recovery/avoid


def make_request(api_id: int, parameter: str | None = None) -> Request:
    req = Request()
    req.header.identity.id = 0
    req.header.identity.api_id = api_id
    req.header.lease.id = 0
    req.header.policy.priority = 0
    req.header.policy.noreply = True
    if parameter is not None:
        req.parameter = parameter
    return req


def clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


class SportBridge(Node):
    def __init__(self) -> None:
        super().__init__("sport_bridge")

        max_lin_env = os.environ.get("GO2_MAX_LINEAR", "").strip()
        max_ang_env = os.environ.get("GO2_MAX_ANGULAR", "").strip()
        self._max_lin = float(max_lin_env) if max_lin_env else None
        self._max_ang = float(max_ang_env) if max_ang_env else None
        cmd_timeout = float(os.environ.get("GO2_CMD_TIMEOUT", "3.0"))
        rate_hz = float(
            os.environ.get(
                "GO2_SPORT_RATE_HZ",
                os.environ.get("GO2_CMD_VEL_HZ", "20"),
            )
        )
        rate_hz = max(1.0, min(100.0, rate_hz))
        stand_on_start = os.environ.get("GO2_SPORT_STAND_ON_START", "1") != "0"
        stand_mode = os.environ.get("GO2_SPORT_STAND_MODE", "balance")
        walk_mode = os.environ.get("GO2_SPORT_WALK_MODE", "free").lower()
        self._auto_recovery = os.environ.get("GO2_SPORT_AUTO_RECOVERY", "1") != "0"  # go2-agri: recovery/avoid
        self._avoid = os.environ.get("GO2_SPORT_AVOID", "0") not in ("", "0")  # go2-agri: recovery/avoid
        self._sent_ids = set()  # go2-agri: recovery/avoid
        self._avoid_pub = self.create_publisher(Request, TOPIC_AVOID, 10)  # go2-agri: recovery/avoid
        self.create_subscription(Response, "/api/obstacles_avoid/response", self._on_reply, 10)  # go2-agri: recovery/avoid
        self.create_subscription(Response, "/api/sport/response", self._on_reply, 10)  # go2-agri: recovery/avoid

        self._cmd_timeout = cmd_timeout
        self._cmd_count = 0
        self._last_cmd_log = time.monotonic()

        self._pub = self.create_publisher(Request, TOPIC_REQUEST, 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._stopped = True
        self._timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._on_timer)

        if stand_on_start:
            self._init_sport_mode(stand_mode, walk_mode)

        cap = "off"
        if self._max_lin is not None or self._max_ang is not None:
            cap = (
                f"lin={self._max_lin:.2f}" if self._max_lin is not None else "lin=off"
            ) + (
                f" ang={self._max_ang:.2f}" if self._max_ang is not None else " ang=off"
            )
        self.get_logger().info(
            f"Robot sport bridge: /cmd_vel -> Move @ {rate_hz:.0f} Hz (cap {cap})"
        )

    def _send(self, api_id: int, parameter: str | None = None) -> None:
        self._pub.publish(make_request(api_id, parameter))

    def _init_sport_mode(self, stand_mode: str, walk_mode: str) -> None:
        """Release RC / app joystick and enable velocity walk (required after RC use)."""
        if stand_mode == "up":
            self._send(API_STAND_UP)
            self.get_logger().info("StandUp")
        else:
            self._send(API_BALANCE_STAND)
            self.get_logger().info("BalanceStand")
        time.sleep(0.4)
        self._send(API_RECOVERY_STAND)
        time.sleep(0.3)
        self._send(API_SWITCH_JOYSTICK, json.dumps({"data": False}))
        self.get_logger().info("SwitchJoystick(false) — put RC aside, do not touch sticks")
        time.sleep(0.3)
        if walk_mode == "classic":
            self._send(API_CLASSIC_WALK, json.dumps({"data": True}))
            self.get_logger().info("ClassicWalk(true)")
        else:
            self._send(API_FREE_WALK)
            self.get_logger().info("FreeWalk")
        time.sleep(0.4)
        self._init_features()  # go2-agri: recovery/avoid

    def _init_features(self) -> None:  # go2-agri: recovery/avoid
        """Fall recovery and obstacle avoidance (sport_bridge_features.py)."""  # go2-agri: recovery/avoid
        if self._auto_recovery:  # go2-agri: recovery/avoid
            self._reply_call(self._pub, API_AUTORECOVERY_SET, {"data": True})  # go2-agri: recovery/avoid
            self.get_logger().info("AutoRecoverySet(true): the dog stands up by itself after a fall")  # go2-agri: recovery/avoid
        if self._avoid:  # go2-agri: recovery/avoid
            self._reply_call(self._avoid_pub, AVOID_SWITCH_SET, {"enable": True})  # go2-agri: recovery/avoid
            time.sleep(0.3)  # go2-agri: recovery/avoid
            self._reply_call(self._avoid_pub, AVOID_USE_API, {"is_remote_commands_from_api": True})  # go2-agri: recovery/avoid
            self.get_logger().info("Obstacle avoidance on: walking through /api/obstacles_avoid (forward only)")  # go2-agri: recovery/avoid
        else:  # go2-agri: recovery/avoid
            # Off unless asked for: the switch survives a reboot, so a session that died  # go2-agri: recovery/avoid
            # without its cleanup would otherwise leave it on (seen 2026-11-12).  # go2-agri: recovery/avoid
            self._reply_call(self._avoid_pub, AVOID_USE_API, {"is_remote_commands_from_api": False})  # go2-agri: recovery/avoid
            self._reply_call(self._avoid_pub, AVOID_SWITCH_SET, {"enable": False})  # go2-agri: recovery/avoid
        time.sleep(0.3)  # go2-agri: recovery/avoid
  # go2-agri: recovery/avoid
    def _reply_call(self, pub, api_id: int, data: dict) -> None:  # go2-agri: recovery/avoid
        req = make_request(api_id, json.dumps(data))  # go2-agri: recovery/avoid
        req.header.identity.id = int(time.time() * 1000) % 2**31  # go2-agri: recovery/avoid
        req.header.policy.noreply = False  # go2-agri: recovery/avoid
        self._sent_ids.add(req.header.identity.id)  # go2-agri: recovery/avoid
        pub.publish(req)  # go2-agri: recovery/avoid
  # go2-agri: recovery/avoid
    def _on_reply(self, msg) -> None:  # go2-agri: recovery/avoid
        # Only replies to this bridge's own requests: other clients on the dog poll  # go2-agri: recovery/avoid
        # the avoidance switch (api 1002) about 10 times a second.  # go2-agri: recovery/avoid
        api = msg.header.identity.api_id  # go2-agri: recovery/avoid
        if msg.header.identity.id in self._sent_ids:  # go2-agri: recovery/avoid
            self._sent_ids.discard(msg.header.identity.id)  # go2-agri: recovery/avoid
            code = msg.header.status.code  # go2-agri: recovery/avoid
            log = self.get_logger().info if code == 0 else self.get_logger().warn  # go2-agri: recovery/avoid
            log(f"reply api {api}: code {code}{' (ok)' if code == 0 else ' (refused)'} {msg.data}")  # go2-agri: recovery/avoid
  # go2-agri: recovery/avoid
    def _move(self, vx: float, vy: float, vyaw: float) -> None:  # go2-agri: recovery/avoid
        if self._avoid:  # go2-agri: recovery/avoid
            self._avoid_pub.publish(make_request(AVOID_MOVE, json.dumps({"x": vx, "y": vy, "yaw": vyaw, "mode": 0})))  # go2-agri: recovery/avoid
        else:  # go2-agri: recovery/avoid
            self._send(API_MOVE, json.dumps({"x": vx, "y": vy, "z": vyaw}))  # go2-agri: recovery/avoid
  # go2-agri: recovery/avoid
    def _halt(self, final: bool = False) -> None:  # go2-agri: recovery/avoid
        if self._avoid:  # go2-agri: recovery/avoid
            self._avoid_pub.publish(make_request(AVOID_MOVE, json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})))  # go2-agri: recovery/avoid
            if final:  # go2-agri: recovery/avoid
                self._reply_call(self._avoid_pub, AVOID_USE_API, {"is_remote_commands_from_api": False})  # go2-agri: recovery/avoid
                self._reply_call(self._avoid_pub, AVOID_SWITCH_SET, {"enable": False})  # go2-agri: recovery/avoid
                self.get_logger().info("Obstacle avoidance off")  # go2-agri: recovery/avoid
        self._send(API_STOP_MOVE)  # go2-agri: recovery/avoid
  # go2-agri: recovery/avoid
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()
        self._cmd_count += 1

    def _on_timer(self) -> None:
        now = time.monotonic()
        if now - self._last_cmd_log >= 5.0:
            vx = float(self._last_cmd.linear.x)
            vyaw = float(self._last_cmd.angular.z)
            self.get_logger().info(
                f"cmd_vel (5s): n={self._cmd_count} last_vx={vx:.2f} vyaw={vyaw:.2f}"
            )
            self._cmd_count = 0
            self._last_cmd_log = now

        age = now - self._last_cmd_time
        stale = self._last_cmd_time == 0.0 or age > self._cmd_timeout
        if stale:
            if not self._stopped:
                self._halt()  # go2-agri: recovery/avoid
                self._stopped = True
            return

        vx = float(self._last_cmd.linear.x)
        vy = float(self._last_cmd.linear.y)
        vyaw = float(self._last_cmd.angular.z)
        if self._max_lin is not None:
            vx = clamp(vx, self._max_lin)
            vy = clamp(vy, self._max_lin)
        if self._max_ang is not None:
            vyaw = clamp(vyaw, self._max_ang)
        self._move(vx, vy, vyaw)  # go2-agri: recovery/avoid
        self._stopped = False


def main() -> None:
    rclpy.init()
    node = SportBridge()

    def interrupt(_signum, _frame):
        # Keep the ROS context alive long enough to attempt StopMove.
        raise KeyboardInterrupt()

    previous_handlers = {sig: signal.signal(sig, interrupt)
                         for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.get_logger().info("StopMove")
                node._halt(final=True)  # go2-agri: recovery/avoid
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    main()
