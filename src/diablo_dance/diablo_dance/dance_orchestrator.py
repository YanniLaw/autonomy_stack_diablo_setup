#!/usr/bin/env python3
"""
diablo_dance: a small, engineering-friendly ROS 2 package to play back a choreography
on Direct Drive Technology's DIABLO robot by publishing motion_msgs/msg/MotionCtrl
to /diablo/MotionCmd.

Two input formats are supported:

1) step-based choreography:
   rate_hz: 50
   steps:
     - name: enter_stand
       duration: 2.0
       mode_mark: true
       mode: {stand_mode: true, height_ctrl_mode: true}
       value: {up: 0.8}

     - name: sway_roll
       duration: 8.0
       pattern:
         type: sine
         channels:
           roll: {amp: 0.25, freq: 0.5, offset: 0.0, phase: 0.0}
           pitch:{amp: 0.10, freq: 1.0, offset: 0.0, phase: 1.57}
       # base value applied + pattern overlay (missing fields default to 0)
       value: {up: 0.8}

2) time-stamped timeline:
   rate_hz: 50
   timeline:
     - t: 0.00
       mode_mark: true
       mode: {...}
       value: {...}
     - t: 0.02
       mode_mark: false
       value: {...}
   Playback holds the latest command at each tick.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

from std_srvs.srv import Trigger, SetBool

try:
    import yaml  # type: ignore
except Exception as e:
    yaml = None  # handled at runtime

from motion_msgs.msg import MotionCtrl  # provided by diablo_ros2 (diablo_interfaces)


# --------------------------- utilities ---------------------------

def _f32(x: Any, default: float = 0.0) -> float:
    try:
        if x is None:
            return float(default)
        return float(x)
    except Exception:
        return float(default)


def _b(x: Any, default: bool = False) -> bool:
    if x is None:
        return default
    if isinstance(x, bool):
        return x
    if isinstance(x, (int, float)):
        return bool(x)
    s = str(x).strip().lower()
    if s in ("1", "true", "yes", "y", "on"):
        return True
    if s in ("0", "false", "no", "n", "off"):
        return False
    return default


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _merge_dict(dst: Dict[str, Any], src: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not src:
        return dst
    for k, v in src.items():
        dst[k] = v
    return dst


# --------------------------- data model ---------------------------

@dataclass
class ChannelSine:
    amp: float
    freq: float
    offset: float = 0.0
    phase: float = 0.0

    def eval(self, t: float) -> float:
        return self.offset + self.amp * math.sin(2.0 * math.pi * self.freq * t + self.phase)


@dataclass
class Keyframe:
    t: float
    value: Dict[str, float]


@dataclass
class Pattern:
    type: str
    channels: Dict[str, Any]  # ChannelSine or list[Keyframe] or scalar overlay
    base: Dict[str, float]

    def eval(self, t: float) -> Dict[str, float]:
        out = dict(self.base)
        if self.type == "sine":
            for name, ch in self.channels.items():
                if isinstance(ch, ChannelSine):
                    out[name] = ch.eval(t)
                else:
                    out[name] = _f32(ch, out.get(name, 0.0))
            return out

        if self.type == "keyframes":
            # Piecewise-linear interpolation for each field
            # channels[field] = list[Keyframe]
            for field, kfs in self.channels.items():
                if not kfs:
                    continue
                if t <= kfs[0].t:
                    out[field] = kfs[0].value[field]
                    continue
                if t >= kfs[-1].t:
                    out[field] = kfs[-1].value[field]
                    continue
                # find enclosing segment
                for i in range(len(kfs) - 1):
                    t0, t1 = kfs[i].t, kfs[i + 1].t
                    if t0 <= t <= t1:
                        v0 = kfs[i].value[field]
                        v1 = kfs[i + 1].value[field]
                        if t1 <= t0:
                            out[field] = v1
                        else:
                            u = (t - t0) / (t1 - t0)
                            out[field] = (1.0 - u) * v0 + u * v1
                        break
            return out

        if self.type == "hold":
            return out

        raise ValueError(f"Unsupported pattern type: {self.type}")


@dataclass
class Step:
    name: str
    duration: float
    mode_mark: bool
    mode: Dict[str, bool]
    value: Dict[str, float]
    pattern: Optional[Pattern] = None


@dataclass
class TimelineCmd:
    t: float
    mode_mark: bool
    mode: Dict[str, bool]
    value: Dict[str, float]


# --------------------------- node ---------------------------

class DiabloDanceOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("diablo_dance_orchestrator")

        self.declare_parameter("choreo_yaml", "")
        self.declare_parameter("cmd_topic", "/diablo/MotionCmd")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("require_yaml", True)

        # conservative clamps (can be overridden)
        self.declare_parameter("clamp.forward", 0.6)
        self.declare_parameter("clamp.left", 0.6)
        self.declare_parameter("clamp.up", 1.0)
        self.declare_parameter("clamp.roll", 0.6)
        self.declare_parameter("clamp.pitch", 0.6)
        self.declare_parameter("clamp.leg_split", 1.0)

        self._cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self._rate_hz = float(self.get_parameter("rate_hz").value)

        self._pub = self.create_publisher(MotionCtrl, self._cmd_topic, 10)

        # services
        self._srv_start = self.create_service(Trigger, "dance/start", self._on_start)
        self._srv_stop = self.create_service(Trigger, "dance/stop", self._on_stop)
        self._srv_pause = self.create_service(SetBool, "dance/pause", self._on_pause)
        self._srv_reload = self.create_service(Trigger, "dance/reload", self._on_reload)

        self._timer = self.create_timer(1.0 / max(self._rate_hz, 1.0), self._on_tick)

        # playback state
        self._playing: bool = False
        self._paused: bool = False
        self._t0: Optional[Time] = None
        self._step_t0: Optional[Time] = None
        self._step_idx: int = 0

        # persistent mode set (kept and attached to subsequent commands for safety)
        self._mode_state: Dict[str, bool] = {
            "pitch_ctrl_mode": False,
            "roll_ctrl_mode": False,
            "height_ctrl_mode": False,
            "stand_mode": False,
            "jump_mode": False,
            "split_mode": False,
        }

        self._steps: List[Step] = []
        self._timeline: List[TimelineCmd] = []
        self._timeline_cursor: int = 0

        # last command values (clamped). Used to stop safely without collapsing height.
        self._last_value: Dict[str, float] = {
            "forward": 0.0,
            "left": 0.0,
            "up": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "leg_split": 0.0,
        }

        self._load_choreo()

        if bool(self.get_parameter("auto_start").value):
            if self._steps or self._timeline:
                self._start()
            else:
                self.get_logger().warn("auto_start is true, but no choreography loaded.")

    # --------------------------- YAML ---------------------------

    def _load_choreo(self) -> None:
        path = str(self.get_parameter("choreo_yaml").value).strip()
        if not path:
            if bool(self.get_parameter("require_yaml").value):
                self.get_logger().warn("No choreo_yaml provided. Provide a YAML path via parameter choreo_yaml.")
            return
        if not os.path.exists(path):
            self.get_logger().error(f"choreo_yaml not found: {path}")
            return
        if yaml is None:
            self.get_logger().error("PyYAML is not available. Install python3-yaml.")
            return

        with open(path, "r") as f:
            doc = yaml.safe_load(f) or {}

        # allow YAML to override rate_hz
        if "rate_hz" in doc:
            self._rate_hz = float(doc["rate_hz"])
            try:
                self._timer.cancel()
            except Exception:
                pass
            self._timer = self.create_timer(1.0 / max(self._rate_hz, 1.0), self._on_tick)

        self._steps = []
        self._timeline = []
        self._timeline_cursor = 0

        if "timeline" in doc and isinstance(doc["timeline"], list):
            for item in doc["timeline"]:
                if not isinstance(item, dict):
                    continue
                self._timeline.append(self._parse_timeline_cmd(item))
            self._timeline.sort(key=lambda c: c.t)
            self.get_logger().info(f"Loaded timeline with {len(self._timeline)} commands from {path}")
            return

        if "steps" in doc and isinstance(doc["steps"], list):
            for item in doc["steps"]:
                if not isinstance(item, dict):
                    continue
                self._steps.append(self._parse_step(item))
            self.get_logger().info(f"Loaded {len(self._steps)} steps from {path}")
            return

        self.get_logger().warn(f"choreo_yaml has no 'steps' or 'timeline': {path}")

    def _parse_mode(self, d: Optional[Dict[str, Any]]) -> Dict[str, bool]:
        out = {}
        if not d:
            return out
        for k, v in d.items():
            out[str(k)] = _b(v, False)
        return out

    def _parse_value(self, d: Optional[Dict[str, Any]]) -> Dict[str, float]:
        out = {}
        if not d:
            return out
        for k, v in d.items():
            out[str(k)] = _f32(v, 0.0)
        return out

    def _parse_pattern(self, d: Optional[Dict[str, Any]], base_value: Dict[str, float]) -> Optional[Pattern]:
        if not d or not isinstance(d, dict):
            return None
        ptype = str(d.get("type", "hold")).strip().lower()

        if ptype == "sine":
            channels: Dict[str, Any] = {}
            chd = d.get("channels", {}) or {}
            if isinstance(chd, dict):
                for field, spec in chd.items():
                    if isinstance(spec, dict):
                        channels[str(field)] = ChannelSine(
                            amp=_f32(spec.get("amp", 0.0)),
                            freq=_f32(spec.get("freq", 0.0)),
                            offset=_f32(spec.get("offset", 0.0)),
                            phase=_f32(spec.get("phase", 0.0)),
                        )
                    else:
                        channels[str(field)] = _f32(spec)
            return Pattern(type="sine", channels=channels, base=dict(base_value))

        if ptype == "keyframes":
            # keyframes: [{t:0.0, value:{roll:0.0,pitch:0.0}}, ...]
            kflist = d.get("keyframes", []) or []
            # regroup by field
            by_field: Dict[str, List[Keyframe]] = {}
            if isinstance(kflist, list):
                for kf in kflist:
                    if not isinstance(kf, dict):
                        continue
                    t = _f32(kf.get("t", 0.0))
                    val = self._parse_value(kf.get("value", {}))
                    for field, vv in val.items():
                        by_field.setdefault(field, []).append(Keyframe(t=t, value={field: vv}))
            # sort and expand to full keyframe lists (each keyframe stores only its field)
            for field, lst in by_field.items():
                lst.sort(key=lambda k: k.t)
            return Pattern(type="keyframes", channels=by_field, base=dict(base_value))

        if ptype == "hold":
            return Pattern(type="hold", channels={}, base=dict(base_value))

        raise ValueError(f"Unsupported pattern type: {ptype}")

    def _parse_step(self, item: Dict[str, Any]) -> Step:
        name = str(item.get("name", f"step_{len(self._steps)}"))
        duration = max(_f32(item.get("duration", 1.0), 1.0), 0.0)
        mode_mark = _b(item.get("mode_mark", False), False)
        mode = self._parse_mode(item.get("mode", {}))
        value = self._parse_value(item.get("value", {}))
        pattern = None
        if "pattern" in item:
            pattern = self._parse_pattern(item.get("pattern"), value)
        return Step(name=name, duration=duration, mode_mark=mode_mark, mode=mode, value=value, pattern=pattern)

    def _parse_timeline_cmd(self, item: Dict[str, Any]) -> TimelineCmd:
        t = max(_f32(item.get("t", 0.0)), 0.0)
        mode_mark = _b(item.get("mode_mark", False), False)
        mode = self._parse_mode(item.get("mode", {}))
        value = self._parse_value(item.get("value", {}))
        return TimelineCmd(t=t, mode_mark=mode_mark, mode=mode, value=value)

    # --------------------------- services ---------------------------

    def _on_start(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._start()
        resp.success = ok
        resp.message = msg
        return resp

    def _on_stop(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._stop()
        resp.success = ok
        resp.message = msg
        return resp

    def _on_pause(self, req: SetBool.Request, resp: SetBool.Response) -> SetBool.Response:
        self._paused = bool(req.data)
        resp.success = True
        resp.message = "paused" if self._paused else "resumed"
        return resp

    def _on_reload(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self._load_choreo()
        resp.success = True
        resp.message = "reloaded"
        return resp

    def _start(self) -> Tuple[bool, str]:
        if not (self._steps or self._timeline):
            return False, "No choreography loaded."
        self._playing = True
        self._paused = False
        now = self.get_clock().now()
        self._t0 = now
        self._step_t0 = now
        self._step_idx = 0
        self._timeline_cursor = 0
        return True, "started"

    def _stop(self) -> Tuple[bool, str]:
        self._playing = False
        self._paused = False
        self._t0 = None
        self._step_t0 = None
        self._step_idx = 0
        self._timeline_cursor = 0
        # Send a safe hold command:
        # - zero velocity/attitude-rate channels
        # - keep the most recently commanded height (up)
        # This avoids suddenly commanding up=0.0 while height_ctrl_mode is enabled.
        safe = dict(self._last_value)
        safe["forward"] = 0.0
        safe["left"] = 0.0
        safe["roll"] = 0.0
        safe["pitch"] = 0.0
        safe["leg_split"] = 0.0
        self._publish_cmd(mode_mark=False, mode={}, value=safe)
        return True, "stopped"

    # --------------------------- publish ---------------------------

    def _limits(self) -> Dict[str, float]:
        return {
            "forward": float(self.get_parameter("clamp.forward").value),
            "left": float(self.get_parameter("clamp.left").value),
            "up": float(self.get_parameter("clamp.up").value),
            "roll": float(self.get_parameter("clamp.roll").value),
            "pitch": float(self.get_parameter("clamp.pitch").value),
            "leg_split": float(self.get_parameter("clamp.leg_split").value),
        }

    def _publish_cmd(self, mode_mark: bool, mode: Dict[str, bool], value: Dict[str, float]) -> None:
        msg = MotionCtrl()
        msg.mode_mark = bool(mode_mark)

        # update persistent mode state iff mode_mark is true
        if msg.mode_mark and mode:
            self._mode_state.update({k: bool(v) for k, v in mode.items()})

        # fill modes (always attach current mode state)
        msg.mode.pitch_ctrl_mode = bool(self._mode_state.get("pitch_ctrl_mode", False))
        msg.mode.roll_ctrl_mode = bool(self._mode_state.get("roll_ctrl_mode", False))
        msg.mode.height_ctrl_mode = bool(self._mode_state.get("height_ctrl_mode", False))
        msg.mode.stand_mode = bool(self._mode_state.get("stand_mode", False))
        msg.mode.jump_mode = bool(self._mode_state.get("jump_mode", False))
        msg.mode.split_mode = bool(self._mode_state.get("split_mode", False))

        # values with clamps
        lim = self._limits()
        msg.value.forward = _clamp(_f32(value.get("forward", 0.0)), -lim["forward"], lim["forward"])
        msg.value.left = _clamp(_f32(value.get("left", 0.0)), -lim["left"], lim["left"])
        msg.value.up = _clamp(_f32(value.get("up", 0.0)), -lim["up"], lim["up"])
        msg.value.roll = _clamp(_f32(value.get("roll", 0.0)), -lim["roll"], lim["roll"])
        msg.value.pitch = _clamp(_f32(value.get("pitch", 0.0)), -lim["pitch"], lim["pitch"])
        msg.value.leg_split = _clamp(_f32(value.get("leg_split", 0.0)), -lim["leg_split"], lim["leg_split"])

        # record last values (after clamps)
        self._last_value = {
            "forward": float(msg.value.forward),
            "left": float(msg.value.left),
            "up": float(msg.value.up),
            "roll": float(msg.value.roll),
            "pitch": float(msg.value.pitch),
            "leg_split": float(msg.value.leg_split),
        }

        self._pub.publish(msg)

    # --------------------------- main tick ---------------------------

    def _on_tick(self) -> None:
        if not self._playing or self._paused:
            return
        if self._t0 is None:
            self._t0 = self.get_clock().now()

        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9

        # timeline playback (priority)
        if self._timeline:
            # advance cursor to latest command <= t
            while (self._timeline_cursor + 1) < len(self._timeline) and self._timeline[self._timeline_cursor + 1].t <= t:
                self._timeline_cursor += 1
            cmd = self._timeline[self._timeline_cursor]
            self._publish_cmd(cmd.mode_mark, cmd.mode, cmd.value)

            # stop at end if t passed last timestamp + 1/rate
            if t >= (self._timeline[-1].t + (1.0 / max(self._rate_hz, 1.0))):
                self._stop()
            return

        # step-based playback
        if not self._steps:
            return

        if self._step_t0 is None:
            self._step_t0 = now

        step = self._steps[self._step_idx]
        t_step = (now - self._step_t0).nanoseconds * 1e-9

        # compute values
        if step.pattern is not None:
            val = step.pattern.eval(t_step)
        else:
            val = dict(step.value)

        self._publish_cmd(step.mode_mark, step.mode, val)

        if t_step >= max(step.duration, 0.0):
            # next step
            self._step_idx += 1
            self._step_t0 = now
            if self._step_idx >= len(self._steps):
                self._stop()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiabloDanceOrchestrator()
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
        # Avoid RCLError if shutdown already happened via SIGINT handler.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
