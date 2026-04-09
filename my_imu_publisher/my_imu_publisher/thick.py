#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy, Image
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
import numpy as np
import math

class VelocitySafetyFilter(Node):
    def __init__(self):
        super().__init__('velocity_safety_filter_restricted')

        # --- Parameters ---
        self.stop_distance = 0.7
        self.slow_distance = 0.7
        self.slow_factor = 0.15
        
        # Depth Parameters
        self.MIN_DIST = 200      
        self.MAX_DIST = 500     
        self.STOP_DIST_LIMIT = 300 
        self.STOP_SENSITIVITY = 2000
        
        # Crop setup
        self.C_TOP, self.C_BOT, self.C_SIDE = 0.40, 0.15, 0.20

        # --- States ---
        self.br = CvBridge()
        self.lidar_stop = False
        self.lidar_slow = False
        self.depth_stop = False
        self.depth_slow = False
        self.kill_active = False
        
        self.twist_joy = None
        self.twist_marker = None
        self.twist_autonomy = None
        
        self.last_joy_time = self.get_clock().now()
        self.last_marker_time = self.get_clock().now()
        self.last_autonomy_time = self.get_clock().now()

        # --- Pub/Sub ---
        self.vel_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_autonomy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_marker', self.cmd_marker_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.timer = self.create_timer(0.05, self.publish_twist)
        self.get_logger().info("🛡️ Safety Filter with MANUAL OVERRIDE (JOY bypass)")

    # -------------------- DEPTH --------------------
    def depth_callback(self, msg):
        try:
            depth_frame = self.br.imgmsg_to_cv2(msg, 'passthrough')
            h, w = depth_frame.shape

            t_y, b_y = int(h * self.C_TOP), int(h * (1.0 - self.C_BOT))
            l_x, r_x = int(w * self.C_SIDE), int(w * (1.0 - self.C_SIDE))
            roi = depth_frame[t_y:b_y, l_x:r_x]

            stop_pix = np.count_nonzero((roi > self.MIN_DIST) & (roi < self.STOP_DIST_LIMIT))
            slow_pix = np.count_nonzero((roi > self.MIN_DIST) & (roi < self.MAX_DIST))

            self.depth_stop = stop_pix > self.STOP_SENSITIVITY
            self.depth_slow = slow_pix > 1000
        except Exception:
            pass

    # -------------------- LIDAR --------------------
    def scan_callback(self, msg):
        min_dist = float('inf')
        offset_rad = math.radians(180.0)
        front_rad = math.radians(30.0)

        for i, dist in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            adj_angle = (angle + offset_rad + math.pi) % (2 * math.pi) - math.pi

            if abs(adj_angle) < front_rad and msg.range_min < dist < msg.range_max:
                min_dist = min(min_dist, dist)

        self.lidar_stop = min_dist < self.stop_distance
        self.lidar_slow = min_dist < self.slow_distance

    # -------------------- INPUT SOURCES --------------------
    def cmd_autonomy_callback(self, msg):
        self.twist_autonomy = msg
        self.last_autonomy_time = self.get_clock().now()

    def cmd_marker_callback(self, msg):
        self.twist_marker = msg
        self.last_marker_time = self.get_clock().now()

    def joy_callback(self, msg):
        # --- CONFIG ---
        DEADMAN_AXIS = 4      # your trigger axis
        AXIS_LINEAR = 1
        AXIS_ANGULAR = 0
        DEADBAND = 0.05

        # --- Deadman (axis-based) ---
        deadman_pressed = len(msg.axes) > DEADMAN_AXIS and msg.axes[DEADMAN_AXIS] < -0.5

        # Kill switch (optional)
        self.kill_active = (len(msg.axes) > 5 and msg.axes[5] < -0.5)

        if not deadman_pressed:
            self.twist_joy = None
            return

        # --- Build Twist ---
        t = Twist()
        t.linear.x = msg.axes[AXIS_LINEAR]
        t.angular.z = msg.axes[AXIS_ANGULAR]

        # --- Deadband ---
        if abs(t.linear.x) < DEADBAND:
            t.linear.x = 0.0
        if abs(t.angular.z) < DEADBAND:
            t.angular.z = 0.0

        self.twist_joy = t
        self.last_joy_time = self.get_clock().now()

    # -------------------- MAIN LOOP --------------------
    def publish_twist(self):
        now = self.get_clock().now()
        timeout_ns = 2000000000  # 2 sec

        active = None
        source = "NONE"

        # --- PRIORITY: JOY > MARKER > AUTO ---
        if self.twist_joy is not None:
            active, source = self.twist_joy, "JOY"

        elif self.twist_marker and (now - self.last_marker_time).nanoseconds < timeout_ns:
            active, source = self.twist_marker, "MARKER"

        elif self.twist_autonomy and (now - self.last_autonomy_time).nanoseconds < timeout_ns:
            active, source = self.twist_autonomy, "AUTO"

        if active is None:
            return

        l_vel = active.linear.x
        a_vel = active.angular.z

        # -------------------- SAFETY --------------------
        if source != "JOY":
            # Apply safety ONLY for AUTO / MARKER

            is_blocked = self.lidar_stop or self.depth_stop
            is_slowed = self.lidar_slow or self.depth_slow

            if is_blocked:
                if l_vel >= 0:
                    l_vel = 0.0
                    a_vel = 0.0
                    self.get_logger().error("🛑 AUTO BLOCKED", throttle_duration_sec=1.0)

            elif is_slowed:
                if l_vel >= 0:
                    l_vel *= self.slow_factor
                    a_vel *= self.slow_factor
                    self.get_logger().warn("⚠️ AUTO SLOWED", throttle_duration_sec=2.0)

        else:
            # --- FULL MANUAL OVERRIDE ---
            self.get_logger().warn("🎮 MANUAL OVERRIDE (SAFETY BYPASSED)", throttle_duration_sec=2.0)

        # -------------------- JOY LIMITS --------------------
        if source == "JOY":
            if self.kill_active:
                l_vel, a_vel = 0.0, 0.0
            l_vel = max(-0.23, min(0.23, l_vel))
            a_vel = max(-0.8, min(0.8, a_vel))
        # -------------------- PUBLISH --------------------
        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = "base_link"
        out.twist.linear.x = l_vel
        out.twist.angular.z = a_vel

        self.vel_pub.publish(out)

# -------------------- MAIN --------------------
def main():
    rclpy.init()
    node = VelocitySafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()