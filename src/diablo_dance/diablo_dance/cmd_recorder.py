#!/usr/bin/env python3
"""
Record MotionCtrl commands (e.g., from teleop) into a YAML timeline suitable for replay
by diablo_dance_orchestrator.

This is the practical way to reproduce a dance from a video:
- you drive the robot with teleop in real time while watching the video
- this node records the resulting /diablo/MotionCmd stream
- you replay it later as an autonomous "dance program"
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_srvs.srv import Trigger, SetBool

try:
    import yaml  # type: ignore
except Exception:
    yaml = None  # handled at runtime

from motion_msgs.msg import MotionCtrl


class DiabloCmdRecorder(Node):
    def __init__(self) -> None:
        super().__init__("diablo_cmd_recorder")

        self.declare_parameter("cmd_topic", "/diablo/MotionCmd")
        self.declare_parameter("out_yaml", "diablo_dance_record.yaml")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("min_dt", 0.0)  # seconds; drop samples closer than this
        self.declare_parameter("auto_start", False)

        self._cmd_topic = str(self.get_parameter("cmd_topic").value)
        self._out_yaml = str(self.get_parameter("out_yaml").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._min_dt = float(self.get_parameter("min_dt").value)

        self._sub = self.create_subscription(MotionCtrl, self._cmd_topic, self._on_msg, 50)

        self._srv_start = self.create_service(Trigger, "record/start", self._on_start)
        self._srv_stop = self.create_service(Trigger, "record/stop", self._on_stop)
        self._srv_enable = self.create_service(SetBool, "record/enable", self._on_enable)

        self._recording: bool = False
        self._t0: Optional[Time] = None
        self._last_t: float = -1.0
        self._timeline: List[Dict[str, Any]] = []

        if bool(self.get_parameter("auto_start").value):
            self._start()

    def _on_start(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._start()
        resp.success = ok
        resp.message = msg
        return resp

    def _on_enable(self, req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        if req.data:
            ok, msg = self._start()
        else:
            ok, msg = self._stop()
        resp.success = ok
        resp.message = msg
        return resp

    def _on_stop(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._stop()
        resp.success = ok
        resp.message = msg
        return resp

    def _start(self):
        if yaml is None:
            return False, "PyYAML not available (install python3-yaml)."
        self._recording = True
        self._t0 = self.get_clock().now()
        self._last_t = -1.0
        self._timeline = []
        return True, "recording"

    def _stop(self):
        if not self._recording:
            return True, "already stopped"
        self._recording = False
        self._flush()
        return True, f"saved to {self._out_yaml}"

    def _flush(self) -> None:
        if yaml is None:
            return
        doc = {
            "rate_hz": self._rate_hz,
            "timeline": self._timeline,
        }
        out_path = self._out_yaml
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        with open(out_path, "w") as f:
            yaml.safe_dump(doc, f, sort_keys=False)

    def _on_msg(self, msg: MotionCtrl) -> None:
        if not self._recording or self._t0 is None:
            return

        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        if self._min_dt > 0.0 and self._last_t >= 0.0 and (t - self._last_t) < self._min_dt:
            return
        self._last_t = t

        item = {
            "t": round(t, 4),
            "mode_mark": bool(msg.mode_mark),
            "mode": {
                "pitch_ctrl_mode": bool(msg.mode.pitch_ctrl_mode),
                "roll_ctrl_mode": bool(msg.mode.roll_ctrl_mode),
                "height_ctrl_mode": bool(msg.mode.height_ctrl_mode),
                "stand_mode": bool(msg.mode.stand_mode),
                "jump_mode": bool(msg.mode.jump_mode),
                "split_mode": bool(msg.mode.split_mode),
            },
            "value": {
                "forward": float(msg.value.forward),
                "left": float(msg.value.left),
                "up": float(msg.value.up),
                "roll": float(msg.value.roll),
                "pitch": float(msg.value.pitch),
                "leg_split": float(msg.value.leg_split),
            },
        }
        self._timeline.append(item)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiabloCmdRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
