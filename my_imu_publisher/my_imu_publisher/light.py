import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
import math

class NeverQuitNavigator(Node):
    def __init__(self):
        super().__init__('never_quit_navigator')

        # Parameters
        self.declare_parameter('jump_threshold', 0.5)  # Meters
        self.declare_parameter('recovery_delay', 3.0)  # Seconds to wait for AMCL
        
        self.jump_thresh = self.get_parameter('jump_threshold').value
        self.recovery_delay = self.get_parameter('recovery_delay').value
        
        # State Tracking
        self.last_good_pose = None
        self.current_goal = None
        self.is_recovering = False

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # --- Subscriptions ---
        # 1. Monitor actual localization
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        # 2. Monitor MANUAL resets from RViz (The Fix)
        self.manual_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.manual_reset_callback, 10)

        # Action Client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('🚀 Never-Quit Navigator Started (Manual Override Enabled)')

    def manual_reset_callback(self, msg):
        """
        Triggered when you manually set a pose in RViz. 
        We accept this as the new truth so the robot doesn't 'pull back'.
        """
        self.get_logger().warn('🛠️ Manual Pose Reset Detected! Updating last known position...')
        self.last_good_pose = msg.pose.pose
        
        # Set recovery flag briefly to ignore the 'jump' caused by your manual click
        self.is_recovering = True
        self.create_timer(1.5, self.clear_recovery_lock)

    def clear_recovery_lock(self):
        self.is_recovering = False
        #elf.get_logger().info('✅ Manual reset synced. Jump detection resumed.')

    def pose_callback(self, msg):
        curr_p = msg.pose.pose.position
        
        if self.last_good_pose is None:
            self.last_good_pose = msg.pose.pose
            return

        # Calculate Distance Jumped
        dist = math.sqrt(
            (curr_p.x - self.last_good_pose.position.x)**2 + 
            (curr_p.y - self.last_good_pose.position.y)**2
        )

        # Logic: Only handle jumps if we aren't currently in a recovery or manual reset state
        if dist > self.jump_thresh and not self.is_recovering:
            self.handle_localization_jump(msg, dist)
        elif not self.is_recovering:
            self.last_good_pose = msg.pose.pose

    def handle_localization_jump(self, msg, dist):
        self.is_recovering = True
        self.get_logger().error(f'⚠️ AMCL JUMP DETECTED ({dist:.2f}m)! Stopping Robot...')

        # 1. Stop physical movement
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        # 2. Reset AMCL to last known good pose
        self.get_logger().info('🔄 Snapping AMCL back to last stable position...')
        reset_msg = PoseWithCovarianceStamped()
        reset_msg.header = msg.header
        reset_msg.pose.pose = self.last_good_pose
        # Standard covariance for a "snapped" reset
        reset_msg.pose.covariance = [0.1] * 36 
        self.initial_pose_pub.publish(reset_msg)

        # 3. Wait for AMCL to settle before re-navigating
        self.create_timer(self.recovery_delay, self.resume_navigation)

    def resume_navigation(self):
        if self.current_goal:
            self.get_logger().info('🟢 Resuming navigation to previous goal...')
            self.send_goal(self.current_goal)
        
        self.is_recovering = False

    def send_goal(self, pose):
        """Standard Nav2 Goal Sender"""
        self.current_goal = pose
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'Sending goal to: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = NeverQuitNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()