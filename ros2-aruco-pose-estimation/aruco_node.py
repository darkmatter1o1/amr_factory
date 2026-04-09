#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import time
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose
from aruco_interfaces.msg import ArucoMarkers
import tf_transformations

class DynamicMarkerRecovery(Node):

    def __init__(self):
        super().__init__('dynamic_marker_recovery')

        # DATA FROM YOUR CAPTURE
        self.marker_db = {
            1: {
                "x": 1.4611, "y": -5.8646, "z": 0.1268,
                "roll": 1.4695, "pitch": 0.0427, "yaw": 1.7695
            }
        }


        self.target_frame = "base_link" 
        self.cov_bad = 0.2
        self.cov_good = 0.15
        self.state = "NORMAL"
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(ArucoMarkers, '/aruco/markers', self.marker_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_marker', 10)
        
        qos_init = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_init)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🔄 Dynamic Recovery Node with Marker 1 Data Started")

    def amcl_cb(self, msg):
        cov = max(msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35])
        if cov > self.cov_bad and self.state == "NORMAL":
            self.state = "SCANNING"
            self.get_logger().warn("⚠️ AMCL Lost! Searching for Marker...")
        elif cov < self.cov_good and self.state != "NORMAL":
            self.state = "NORMAL"
            self.get_logger().info("✅ Localization Stable.")

    def marker_cb(self, msg):
        if self.state != "SCANNING":
            return

        for marker_id, marker_pose in zip(msg.marker_ids, msg.poses):
            if marker_id in self.marker_db:
                # Use Time() to avoid extrapolation errors
                calculated_pose = self.calculate_robot_pose(marker_id, marker_pose, msg.header.frame_id)
                if calculated_pose:
                    self.active_robot_pose = calculated_pose
                    self.state = "STOPPING"
                    self.stop_start_time = time.time()
                    return

    def calculate_robot_pose(self, marker_id, marker_in_camera, camera_frame):
        try:
            # Using Time() ensures we get the latest transform without waiting for exact sync
            t_base_cam = self.tf_buffer.lookup_transform(self.target_frame, camera_frame, Time())
            
            marker_stamped = PoseStamped()
            marker_stamped.pose = marker_in_camera
            marker_in_base = do_transform_pose(marker_stamped.pose, t_base_cam)

            T_map_marker = self.get_matrix_from_pose(self.marker_db[marker_id])
            T_base_marker = self.get_matrix_from_pose(marker_in_base)
            
            T_map_base = tf_transformations.concatenate_matrices(
                T_map_marker, 
                tf_transformations.inverse_matrix(T_base_marker)
            )
            
            return self.get_pose_from_matrix(T_map_base)
        except Exception as e:
            self.get_logger().debug(f"Transform wait: {e}")
            return None

    def get_matrix_from_pose(self, p):
        if isinstance(p, dict):
            t = (p['x'], p['y'], p['z'])
            r = tf_transformations.quaternion_from_euler(p['roll'], p['pitch'], p['yaw'])
        else:
            t = (p.position.x, p.position.y, p.position.z)
            r = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return tf_transformations.concatenate_matrices(tf_transformations.translation_matrix(t), tf_transformations.quaternion_matrix(r))

    def get_pose_from_matrix(self, m):
        trans = tf_transformations.translation_from_matrix(m)
        quat = tf_transformations.quaternion_from_matrix(m)
        p = PoseStamped().pose
        p.position.x, p.position.y, p.position.z = trans[0], trans[1], trans[2]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat[0], quat[1], quat[2], quat[3]
        return p

    def control_loop(self):
        if self.state == "SCANNING":
            cmd = Twist()
            cmd.angular.z = 0.1 # Slow rotation to find marker
            self.cmd_pub.publish(cmd)
        elif self.state == "STOPPING":
            self.cmd_pub.publish(Twist())
            if time.time() - self.stop_start_time > 1.0: # Pause to let sensors settle
                self.publish_initialpose()
                self.state = "NORMAL"

    def publish_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self.active_robot_pose
        # Standard AMCL re-init covariance
        for i in [0, 7, 35]: msg.pose.covariance[i] = 0.05
        self.pose_pub.publish(msg)
        self.get_logger().info(f"🎯 RELOCALIZED: Robot is now at {self.active_robot_pose.position.x:.2f}, {self.active_robot_pose.position.y:.2f}")

def main():
    rclpy.init()
    rclpy.spin(DynamicMarkerRecovery())
    rclpy.shutdown()

if __name__ == '__main__':
    main()