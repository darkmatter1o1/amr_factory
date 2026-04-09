#!/usr/bin/env python3
import time
import json
import argparse
import cv2
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import paho.mqtt.client as mqtt

class GoalStatus(IntEnum):
    STATUS_EXECUTING = 2
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED  = 5
    STATUS_ABORTED   = 6

class RobotAlertPublisher(Node):
    def __init__(self, args):
        super().__init__('robot_alert_publisher')

        # Configuration
        self.robot_name = args.robot
        self.broker_host = args.broker
        self.broker_port = args.broker_port
        self.alert_topic = f"{args.topic}/text"
        self.image_topic = f"{args.topic}/image"
        self.alert_cooldown = args.cooldown

        # State tracking
        self.is_navigating = False
        self.last_alert_time = 0.0
        self.last_goal_id = None
        self.latest_cv_image = None
        self.bridge = CvBridge()

        # MQTT setup
        self.mqtt_client = mqtt.Client(client_id=f"jetson_{self.robot_name}_{int(time.time())}")
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_connected = False
        
        try:
            self.mqtt_client.connect_async(self.broker_host, self.broker_port, keepalive=30)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connection failed: {e}")

        # ROS2 subscriptions
        status_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        # 1. Monitor Nav2 Status
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._nav2_status_callback, status_qos)
        
        # 2. Monitor Camera (Change '/camera/image_raw' to your actual topic)
        self.create_subscription(Image, args.camera_topic, self._camera_callback, 10)

        self.get_logger().info(f"Monitoring {args.camera_topic} for snapshots on failure...")

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0: self.mqtt_connected = True

    def _camera_callback(self, msg):
        """Continuously cache the latest frame."""
        try:
            # Convert ROS Image to OpenCV BGR
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def _nav2_status_callback(self, msg: GoalStatusArray):
        if not msg.status_list: return
        latest = msg.status_list[-1]
        goal_id = ''.join(f'{b:02x}' for b in latest.goal_info.goal_id.uuid[:4])
        
        if latest.status == GoalStatus.STATUS_EXECUTING:
            self.is_navigating = True
            self.last_goal_id = goal_id
        elif latest.status == GoalStatus.STATUS_ABORTED:
            if self.last_goal_id == goal_id and self.is_navigating:
                self._trigger_alert_with_image(goal_id)
            self.is_navigating = False

    def _trigger_alert_with_image(self, goal_id):
        now = time.time()
        if now - self.last_alert_time < self.alert_cooldown: return
        if not self.mqtt_connected: return

        # 1. Send Text Alert
        msg_payload = {
            "robot": self.robot_name,
            "event": "navigation_failed",
            "message": f"Robot {self.robot_name} failed goal {goal_id}.",
            "timestamp": now
        }
        self.mqtt_client.publish(self.alert_topic, json.dumps(msg_payload), qos=1)

        # 2. Send Image Snapshot
        if self.latest_cv_image is not None:
            # Compress image to JPEG to save bandwidth
            success, buffer = cv2.imencode(".jpg", self.latest_cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if success:
                self.mqtt_client.publish(self.image_topic, buffer.tobytes(), qos=1)
                self.get_logger().info(f"📤 Alert + Snapshot sent for goal {goal_id}")
        
        self.last_alert_time = now

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, default='vbot1')
    parser.add_argument('--broker', type=str, default='broker.hivemq.com')
    parser.add_argument('--broker-port', type=int, default=1883)
    parser.add_argument('--topic', type=str, default='vbot1_avinash/alerts')
    parser.add_argument('--camera-topic', type=str, default='/camera/color/image_raw')
    parser.add_argument('--cooldown', type=float, default=10.0)

    args = parser.parse_args()
    rclpy.init()
    node = RobotAlertPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()