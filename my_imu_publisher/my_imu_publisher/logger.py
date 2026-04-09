#!/usr/bin/env python3
"""
AMR Session Logger Node  —  vbot1_v2 / ROS2 Humble
====================================================
Logs every operational event to a per-session CSV file:
  • Robot power-on / node-start timestamp
  • Navigation round start / end (time, distance, duration)
  • Battery level at start/end of each round + drain %
  • Total odometric distance accumulated in session
  • 30-second heartbeat rows
  • Goal CANCELLED or ABORTED mid-nav → JPEG snapshot with
    metadata overlay saved to <log_dir>/snapshots/

CSV columns:
  timestamp | event_type | session_start | round_number |
  round_start_time | round_end_time | round_duration_sec |
  round_distance_m | battery_start_pct | battery_end_pct |
  battery_drain_pct | total_distance_m | session_uptime_sec |
  cancel_reason | snapshot_path | notes

Parameters (all ros2 params):
  log_dir            default: ~/amr_logs
  camera_topic       default: /camera/color/image_raw
  battery_topic      default: /battery_state
  odom_topic         default: /odom
  nav_status_topic   default: /navigate_to_pose/_action/status
  heartbeat_interval default: 30.0  (seconds)
"""

import os
import csv
import math
import threading
from datetime import datetime

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CSV_HEADERS = [
    "timestamp",
    "event_type",
    "session_start",
    "round_number",
    "round_start_time",
    "round_end_time",
    "round_duration_sec",
    "round_distance_m",
    "battery_start_pct",
    "battery_end_pct",
    "battery_drain_pct",
    "total_distance_m",
    "session_uptime_sec",
    "cancel_reason",
    "snapshot_path",
    "notes",
]

FONT = cv2.FONT_HERSHEY_SIMPLEX


class AMRSessionLogger(Node):
    # ------------------------------------------------------------------
    # Init
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__("amr_session_logger")

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter("log_dir", os.path.expanduser("~/amr_logs"))
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("battery_topic", "/battery_info")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "nav_status_topic", "/navigate_to_pose/_action/status"
        )
        self.declare_parameter("heartbeat_interval", 30.0)

        log_dir = self.get_parameter("log_dir").value
        self._log_dir = os.path.expanduser(log_dir)
        self._snap_dir = os.path.join(self._log_dir, "snapshots")
        os.makedirs(self._snap_dir, exist_ok=True)

        # ── Session bookkeeping ──────────────────────────────────────────
        self._session_start = datetime.now()
        self._session_tag = self._session_start.strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(
            self._log_dir, f"session_{self._session_tag}.csv"
        )
        self._init_csv()

        # ── State ────────────────────────────────────────────────────────
        self._round_count: int = 0
        self._active_goals: dict = {}          # gid -> {round, start_time, start_bat, start_dist}
        self._prev_statuses: dict = {}         # gid -> GoalStatus int

        self._total_distance: float = 0.0
        self._prev_pos: tuple | None = None

        self._battery_pct: float | None = None
        self._session_start_battery: float | None = None

        self._latest_image: Image | None = None
        self._image_lock = threading.Lock()
        self._bridge = CvBridge()

        # ── QoS presets ──────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self._odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_cb,
            sensor_qos,
        )
        self._bat_sub = self.create_subscription(
            BatteryState,
            self.get_parameter("battery_topic").value,
            self._battery_cb,
            sensor_qos,
        )
        self._nav_sub = self.create_subscription(
            GoalStatusArray,
            self.get_parameter("nav_status_topic").value,
            self._nav_status_cb,
            reliable_qos,
        )
        self._cam_sub = self.create_subscription(
            Image,
            self.get_parameter("camera_topic").value,
            self._image_cb,
            sensor_qos,
        )

        # ── Heartbeat timer ──────────────────────────────────────────────
        interval = float(self.get_parameter("heartbeat_interval").value)
        self._heartbeat_timer = self.create_timer(interval, self._heartbeat_cb)

        # ── Log session start row ────────────────────────────────────────
        self._write_row(
            "SESSION_START",
            notes=f"Robot powered on. Log: {self._csv_path}",
            round_start_time=self._session_tag,
        )
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  AMR Session Logger  STARTED\n"
            f"  Session tag : {self._session_tag}\n"
            f"  CSV         : {self._csv_path}\n"
            f"  Snapshots   : {self._snap_dir}\n"
            f"{'='*60}"
        )

    # ------------------------------------------------------------------
    # CSV helpers
    # ------------------------------------------------------------------
    def _init_csv(self):
        with open(self._csv_path, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_HEADERS).writeheader()

    def _write_row(self, event_type: str, **kwargs):
        now = datetime.now()
        uptime = (now - self._session_start).total_seconds()
        row = {h: "" for h in CSV_HEADERS}
        row.update(
            {
                "timestamp": now.strftime("%Y-%m-%d %H:%M:%S"),
                "event_type": event_type,
                "session_start": self._session_tag,
                "total_distance_m": f"{self._total_distance:.3f}",
                "session_uptime_sec": f"{uptime:.1f}",
                "battery_end_pct": (
                    f"{self._battery_pct:.1f}"
                    if self._battery_pct is not None
                    else ""
                ),
            }
        )
        row.update({k: v for k, v in kwargs.items() if k in CSV_HEADERS})
        with open(self._csv_path, "a", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_HEADERS).writerow(row)

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        cur = (pos.x, pos.y)
        if self._prev_pos is not None:
            dx = cur[0] - self._prev_pos[0]
            dy = cur[1] - self._prev_pos[1]
            d = math.hypot(dx, dy)
            if d > 0.002:          # ignore sub-millimetre jitter
                self._total_distance += d
        self._prev_pos = cur

    def _battery_cb(self, msg: BatteryState):
        # BatteryState.percentage is 0.0–1.0; multiply to get %
        pct = msg.percentage * 100.0
        self._battery_pct = pct
        if self._session_start_battery is None:
            self._session_start_battery = pct
            self.get_logger().info(f"[BATTERY] Initial charge: {pct:.1f}%")

    def _image_cb(self, msg: Image):
        with self._image_lock:
            self._latest_image = msg

    def _nav_status_cb(self, msg: GoalStatusArray):
        cur: dict = {}
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()
            cur[gid] = s.status

        for gid, status in cur.items():
            prev = self._prev_statuses.get(gid, GoalStatus.STATUS_UNKNOWN)

            if (
                status == GoalStatus.STATUS_EXECUTING
                and prev != GoalStatus.STATUS_EXECUTING
            ):
                self._nav_started(gid)

            elif (
                status == GoalStatus.STATUS_SUCCEEDED
                and prev == GoalStatus.STATUS_EXECUTING
            ):
                self._nav_succeeded(gid)

            elif status == GoalStatus.STATUS_CANCELED and prev in (
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_ACCEPTED,
            ):
                self._nav_canceled(gid)

            elif status == GoalStatus.STATUS_ABORTED and prev in (
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_ACCEPTED,
            ):
                self._nav_aborted(gid)

        self._prev_statuses = cur

    # ------------------------------------------------------------------
    # Navigation lifecycle
    # ------------------------------------------------------------------
    def _nav_started(self, gid: str):
        self._round_count += 1
        self._active_goals[gid] = {
            "round": self._round_count,
            "start_time": datetime.now(),
            "start_battery": self._battery_pct,
            "start_distance": self._total_distance,
        }
        bat_str = (
            f"{self._battery_pct:.1f}%"
            if self._battery_pct is not None
            else "unknown"
        )
        self.get_logger().info(
            f"[Round {self._round_count}] ▶  Navigation STARTED  |  Battery: {bat_str}"
        )
        self._write_row(
            "NAV_STARTED",
            round_number=self._round_count,
            round_start_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            battery_start_pct=(
                f"{self._battery_pct:.1f}" if self._battery_pct is not None else ""
            ),
        )

    def _nav_succeeded(self, gid: str):
        self._finalise(gid, "NAV_COMPLETED")

    def _nav_canceled(self, gid: str):
        snap = self._take_snapshot(gid, "CANCELED")
        self._finalise(
            gid,
            "NAV_CANCELED",
            cancel_reason="Goal canceled by operator or system",
            snapshot_path=snap,
        )

    def _nav_aborted(self, gid: str):
        snap = self._take_snapshot(gid, "ABORTED")
        self._finalise(
            gid,
            "NAV_ABORTED",
            cancel_reason="Navigation aborted — planner / controller failure",
            snapshot_path=snap,
        )

    def _finalise(
        self,
        gid: str,
        event_type: str,
        cancel_reason: str = "",
        snapshot_path: str = "",
    ):
        meta = self._active_goals.pop(gid, None)
        if meta is None:
            self.get_logger().warn(f"_finalise called for unknown goal {gid}")
            return

        now = datetime.now()
        duration = (now - meta["start_time"]).total_seconds()
        round_dist = self._total_distance - meta["start_distance"]

        b_start = meta["start_battery"]
        b_end = self._battery_pct
        b_drain = (
            f"{b_start - b_end:.2f}"
            if (b_start is not None and b_end is not None)
            else ""
        )

        self.get_logger().info(
            f"[Round {meta['round']}] ■  {event_type}  |  "
            f"Duration: {duration:.1f}s  |  "
            f"Distance: {round_dist:.2f} m  |  "
            + (f"Battery drain: {b_drain}%" if b_drain else "")
        )

        self._write_row(
            event_type,
            round_number=meta["round"],
            round_start_time=meta["start_time"].strftime("%Y-%m-%d %H:%M:%S"),
            round_end_time=now.strftime("%Y-%m-%d %H:%M:%S"),
            round_duration_sec=f"{duration:.1f}",
            round_distance_m=f"{round_dist:.3f}",
            battery_start_pct=f"{b_start:.1f}" if b_start is not None else "",
            battery_end_pct=f"{b_end:.1f}" if b_end is not None else "",
            battery_drain_pct=b_drain,
            cancel_reason=cancel_reason,
            snapshot_path=snapshot_path,
        )

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------
    def _take_snapshot(self, gid: str, reason: str) -> str:
        with self._image_lock:
            img_msg = self._latest_image

        if img_msg is None:
            self.get_logger().warn("Snapshot requested but no camera frame available.")
            return ""

        try:
            frame = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return ""

        meta = self._active_goals.get(gid, {})
        rnd = meta.get("round", self._round_count)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bat_tag = (
            f"{self._battery_pct:.0f}pct"
            if self._battery_pct is not None
            else "UNK"
        )
        filename = f"snap_{reason}_R{rnd:03d}_{ts}_BAT{bat_tag}_D{self._total_distance:.1f}m.jpg"
        filepath = os.path.join(self._snap_dir, filename)

        # ── Overlay metadata ─────────────────────────────────────────────
        h, w = frame.shape[:2]
        overlay = frame.copy()
        # semi-transparent dark banner at top
        cv2.rectangle(overlay, (0, 0), (w, 220), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        lines = [
            f"EVENT      : {reason}",
            f"Session    : {self._session_tag}",
            f"Round      : {rnd}",
            f"Timestamp  : {ts}",
            f"Battery    : {self._battery_pct:.1f}%" if self._battery_pct is not None else "Battery    : N/A",
            f"Total Dist : {self._total_distance:.2f} m",
            f"Uptime     : {(datetime.now()-self._session_start).total_seconds():.0f} s",
        ]
        colours = {
            "CANCELED": (0, 80, 255),    # orange-red
            "ABORTED": (0, 0, 220),       # red
        }
        colour = colours.get(reason, (0, 220, 80))

        for i, line in enumerate(lines):
            y = 28 + i * 27
            # shadow
            cv2.putText(frame, line, (12, y + 1), FONT, 0.62,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, line, (12, y), FONT, 0.62,
                        colour, 1, cv2.LINE_AA)

        cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.get_logger().info(f"[SNAPSHOT] saved → {filepath}")
        return filepath

    # ------------------------------------------------------------------
    # Heartbeat
    # ------------------------------------------------------------------
    def _heartbeat_cb(self):
        session_drain = (
            f"{self._session_start_battery - self._battery_pct:.2f}"
            if (
                self._session_start_battery is not None
                and self._battery_pct is not None
            )
            else ""
        )
        self._write_row(
            "STATUS_HEARTBEAT",
            notes=(
                f"Rounds completed: {self._round_count}  |  "
                f"Active goals: {len(self._active_goals)}  |  "
                f"Session drain: {session_drain}%"
            ),
            battery_drain_pct=session_drain,
        )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        session_drain = (
            f"{self._session_start_battery - self._battery_pct:.2f}"
            if (
                self._session_start_battery is not None
                and self._battery_pct is not None
            )
            else ""
        )
        self._write_row(
            "SESSION_END",
            notes=(
                f"Total rounds: {self._round_count}  |  "
                f"Total distance: {self._total_distance:.2f} m  |  "
                f"Session battery drain: {session_drain}%"
            ),
            battery_drain_pct=session_drain,
        )
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  SESSION END\n"
            f"  Rounds      : {self._round_count}\n"
            f"  Distance    : {self._total_distance:.2f} m\n"
            f"  Battery used: {session_drain}%\n"
            f"  CSV saved   : {self._csv_path}\n"
            f"{'='*60}"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AMRSessionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()