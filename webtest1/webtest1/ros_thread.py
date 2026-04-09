import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image, Joy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
import cv2
import numpy as np
import threading

battery_level_data = {"level": None}
robot_position = {"x": 0.0, "y": 0.0}
latest_depth_frame = None
depth_lock = threading.Lock()


class AMRInterface(Node):
    def __init__(self):
        super().__init__('amr_web_interface')

        self.battery_sub = self.create_subscription(
            Float32, '/battery_level', self.battery_callback, qos_profile_sensor_data)

        self.goal_pub = self.create_publisher(
            Int32, '/selected_goal1', 10)

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/diff_cont/cmd_vel', 10)

        self.position = {"x": 0.0, "y": 0.0}
        self.path = []
        self.create_subscription(Odometry, "/diff_cont/odom", self.odom_callback, 10)

        # Time trackers for topic health
        self.last_robot_time = 0
        self.last_scan_time = 0
        self.last_map_time = 0
        self.last_costmap_time = 0

        # Topic health subscriptions
        self.create_subscription(Odometry, '/diff_cont/odom', self.robot_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # Using published_footprint as the perfect lightweight Nav2 heartbeat (publishes continuously)
        self.create_subscription(PolygonStamped, '/global_costmap/published_footprint', self.costmap_callback, 10)

        # Depth camera subscriber
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.emergency_active = False
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.emergency_timer = self.create_timer(0.5, self.cancel_bt_nav)  # 2 Hz

        # Joystick override detection
        self.last_joy_time = 0
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Obstacle detection
        self.obstacle_detected = False
        self.obstacle_first_detected_time = 0
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)






    # Callback for battery level
    def battery_callback(self, msg):
        battery_level_data["level"] = round(msg.data, 2)

    def depth_callback(self, msg):
        global latest_depth_frame
        try:
            if '16UC1' in msg.encoding or '16' in msg.encoding:
                dtype = np.uint16 if 'U' in msg.encoding else np.int16
                depth_img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
            elif '32FC1' in msg.encoding:
                depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            else:
                depth_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)

            # Ignore 0s for scaling (0 usually means invalid depth)
            valid_depths = depth_img[depth_img > 0]
            if len(valid_depths) > 0:
                min_depth = valid_depths.min()
                max_depth = valid_depths.max()
                if max_depth > min_depth:
                    # Normalizing for best visual representation using minmax mapping
                    # To be super futuristic, let's artificially bound max to 5.0m or 5000mm to avoid flat depth map
                    upper_bound = 5000 if '16' in msg.encoding else 5.0
                    max_d = min(max_depth, upper_bound)
                    # Clip so we don't skew the whole image due to one glitchy pixel far away
                    depth_clipped = np.clip(depth_img, min_depth, max_d)
                    
                    depth_normalized = cv2.normalize(depth_clipped, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    
                    # For a truly "futuristic" feel, let's invert the map (so close objects are red/hot, far are blue/cold)
                    depth_normalized = 255 - depth_normalized
                else:
                    depth_normalized = np.zeros((msg.height, msg.width), dtype=np.uint8)
            else:
                depth_normalized = np.zeros((msg.height, msg.width), dtype=np.uint8)

            # Apply futuristic colormap
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)
            
            # Encode as JPEG
            ret, buffer = cv2.imencode('.jpg', depth_colored, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            if ret:
                with depth_lock:
                    latest_depth_frame = buffer.tobytes()

        except Exception as e:
            self.get_logger().error(f"Error processing depth: {e}")

    # Callback for odometry
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # Calculate yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.position = {"x": x, "y": y, "yaw": yaw}
        self.path.append({"x": x, "y": y})
        if len(self.path) > 100:
            self.path.pop(0)

    def get_position(self):
        return self.position

    def get_path(self):
        return self.path



    # Publish goal ID
    def publish_goal(self, goal_number: int):
        msg = Int32()
        msg.data = goal_number
        self.goal_pub.publish(msg)

    # Publish manual twist command
    def publish_twist(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(angular_z)

        self.cmd_vel_pub.publish(msg)

    # Topic health callbacks
    def scan_callback(self, msg):
        self.last_scan_time = time.time()
    
    def robot_callback(self, msg):
        self.last_robot_time = time.time()

    def map_callback(self, msg):
        self.last_map_time = time.time()

    def costmap_callback(self, msg):
        self.last_costmap_time = time.time()

    # Health check logic
    def get_topic_health(self):
        now = time.time()
        return {
            "robot": (now - self.last_robot_time) < 2.0,  # You can customize this further
            "map": (now - self.last_map_time) < 10.0,
            "navigation": (now - self.last_costmap_time) < 5.0,
            "lidar": (now - self.last_scan_time) < 2.0
        }
    def get_all_topics(self):
        try:
            topics = self.get_topic_names_and_types()
            result = []
            for name, types in topics:
                result.append({"name": name, "types": types})
            return result
        except Exception as e:
            self.get_logger().error(f"Failed to get topics: {e}")
            return []

    def get_system_state(self):
        return {
            "emergency_active": getattr(self, "emergency_active", False)
        }

    def publish_initial_pose(self, x: float, y: float, yaw_deg: float):
        pose_msg = PoseWithCovarianceStamped()
        now = self.get_clock().now().to_msg()

    # Header
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "map"

    # Pose position
        pose_msg.pose.pose.position.x = 0.2303389901514630
        pose_msg.pose.pose.position.y = 0.1474473721628936
        pose_msg.pose.pose.position.z = 0.0

    # Orientation: convert yaw to quaternion
        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        pose_msg.pose.pose.orientation.z = 0.0349955634668762
        pose_msg.pose.pose.orientation.w = 0.9993874676708908

    # Covariance: small uncertainty
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
        ]

        self.initial_pose_pub.publish(pose_msg)
        return {"x": x, "y": y, "yaw_deg": yaw_deg}

    def cancel_bt_nav(self):
        if self.emergency_active:
            msg = Bool()
            msg.data = True
            self.emergency_pub.publish(msg)
            self.get_logger().warn("🚨 Emergency active: Publishing True to /emergency_stop")


    def activate_emergency(self):
        self.emergency_active = True

    def cancel_current_goal(self):
        """Cancel all Nav2 goals via the cancel_goal service."""
        from action_msgs.srv import CancelGoal
        from action_msgs.msg import GoalInfo

        if not hasattr(self, '_cancel_client'):
            self._cancel_client = self.create_client(
                CancelGoal, '/navigate_to_pose/_action/cancel_goal'
            )

        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Nav2 cancel service not available")
            return

        request = CancelGoal.Request()
        # Empty goal_info means cancel ALL goals
        request.goal_info = GoalInfo()
        future = self._cancel_client.call_async(request)
        self.get_logger().info("Nav2 cancel request sent to navigate_to_pose")

    def joy_callback(self, msg):
        # Only flag override if sticks (0-3) or d-pad (6-7) have input
        # Axes 4,5 are triggers that idle at 1.0 — ignore them
        stick_axes = [msg.axes[i] for i in [0, 1, 2, 3, 6, 7] if i < len(msg.axes)]
        if any(abs(axis) > 0.05 for axis in stick_axes):
            self.last_joy_time = time.time()

    def obstacle_callback(self, msg):
        if msg.data:
            if not self.obstacle_detected:
                self.obstacle_first_detected_time = time.time()
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
            self.obstacle_first_detected_time = 0

    def get_safety_status(self):
        now = time.time()
        joystick_active = (now - self.last_joy_time) < 2.0 if self.last_joy_time > 0 else False
        obstacle_duration = 0
        if self.obstacle_detected and self.obstacle_first_detected_time > 0:
            obstacle_duration = int(now - self.obstacle_first_detected_time)
        return {
            "joystick_override": joystick_active,
            "obstacle_detected": self.obstacle_detected,
            "obstacle_duration": obstacle_duration
        }
