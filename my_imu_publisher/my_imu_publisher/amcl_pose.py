import rclpy
from rclpy.node import Node
import json
import os
import tempfile


from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

POSE_FILE = "/home/jetson/.ros/amcl_pose.json"  # Safer writable path

class PoseSaverLoader(Node):
    def __init__(self):
        super().__init__('pose_saver_loader')

        self.latest_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.timer = self.create_timer(0.2, self.save_pose_timer)

        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Load and schedule publishing of initial pose
        self.initial_pose = self.load_pose_from_file()
        self.publish_count = 0
        if self.initial_pose:
            self.init_pub_timer = self.create_timer(1.0, self.publish_initial_pose)

    def pose_callback(self, msg):
        self.latest_pose = msg

    def save_pose_timer(self):
        if not self.latest_pose:
            return

        data = {
            "position": {
                "x": self.latest_pose.pose.pose.position.x,
                "y": self.latest_pose.pose.pose.position.y,
                "z": self.latest_pose.pose.pose.position.z
            },
            "orientation": {
                "x": self.latest_pose.pose.pose.orientation.x,
                "y": self.latest_pose.pose.pose.orientation.y,
                "z": self.latest_pose.pose.pose.orientation.z,
                "w": self.latest_pose.pose.pose.orientation.w
            },
            "covariance": list(self.latest_pose.pose.covariance)
        }

        try:
        # Write to a temporary file in the same directory
            tmp_dir = os.path.dirname(POSE_FILE)
            os.makedirs(tmp_dir, exist_ok=True)

            with tempfile.NamedTemporaryFile('w', delete=False, dir=tmp_dir) as tmp_file:
                json.dump(data, tmp_file)
                tmp_file.flush()           # Force Python to flush to OS
                os.fsync(tmp_file.fileno())  # Force OS to flush to disk

                temp_path = tmp_file.name

        # Atomically replace the original file with the temp file
            os.replace(temp_path, POSE_FILE)
            #self.get_logger().info("Safely wrote AMCL pose to disk")

        except Exception as e:
            self.get_logger().error(f"Failed to save pose: {e}")
        

    def load_pose_from_file(self):
        if not os.path.exists(POSE_FILE):
            self.get_logger().warn("No previous pose file found.")
            return None

        try:
            with open(POSE_FILE, 'r') as f:
                content = f.read().strip()
                if not content:
                    self.get_logger().warn("Pose file is empty.")
                    return None
                data = json.loads(content)
        except Exception as e:
            self.get_logger().error(f"Failed to load pose: {e}")
            return None

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = data["position"]["x"]
        msg.pose.pose.position.y = data["position"]["y"]
        msg.pose.pose.position.z = data["position"]["z"]
        msg.pose.pose.orientation.x = data["orientation"]["x"]
        msg.pose.pose.orientation.y = data["orientation"]["y"]
        msg.pose.pose.orientation.z = data["orientation"]["z"]
        msg.pose.pose.orientation.w = data["orientation"]["w"]
        msg.pose.covariance = data["covariance"]

        self.get_logger().info("Loaded pose from file and will publish to /initialpose.")
        return msg

    def publish_initial_pose(self):
        if self.publish_count < 3:
            self.initial_pose.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(self.initial_pose)
            self.get_logger().info(f"Published initial pose ({self.publish_count+1}/3)")
            self.publish_count += 1
        else:
            self.init_pub_timer.cancel()
            self.get_logger().info("Stopped publishing initial pose.")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSaverLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
