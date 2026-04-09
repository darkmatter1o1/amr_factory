#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus  # Required for Humble success checks

class Nav2GoalSequencer(Node):
    def __init__(self):
        super().__init__('goal_sequencer')

        # Subscribers
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(Int32, '/selected_goal1', self.goal_sequence_callback, 10)
        self.create_subscription(Float32, '/battery_level', self.battery_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)

        # Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # States
        self.goal_queue = []
        self.current_goal_id = None
        self._goal_handle = None
        self.sequence_running = False
        self.pause = False
        self.low_battery = False
        self.delay_between_goals = 0.3
        self.timer = None
        self.resume_delay_timer = None

        # Define goals (ID: Pose)
        self.goals = {
            0: self.make_pose_q(2.29, 0.29, -0.01, 0.99),
            1: self.make_pose_q(0.29, 23.77, 0.78, 0.706),
            2: self.make_pose_q(-1.138, 23.77,  0.99, 0.0167),
            3: self.make_pose_q(11.46, 5.82, 0.70, 0.70),
            
            4: self.make_pose_q(0.06, 23.73, 0.08006, 0.996),
            5: self.make_pose_q(1.55, 0.44, -0.78, 0.61),
            6: self.make_pose_q(0.0477, 0.0693, 0.999, 0.0189)
        }
        # Define sequences
        self.sequences = {
            1: [0, 1, 2],
            2: [4, 5, 6],
            #3: [1, 3, 4]
        }

    def make_pose_q(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    # ---------------------- Callbacks ----------------------
    def battery_callback(self, msg):
        battery = msg.data
        if battery < 10 and not self.low_battery:
            self.low_battery = True
            self.get_logger().warning(f"🔋 Low battery ({battery}%). Cancelling...")
            self.cancel_current_goal()
        elif battery > 35 and self.low_battery:
            self.low_battery = False
            self.get_logger().info("🔋 Battery recovered. Resuming in 60s...")
            self.resume_delay_timer = self.create_timer(60.0, self.resume_sequence_after_battery)

    def obstacle_callback(self, msg):
        if msg.data:
            self.get_logger().warning("🚧 Obstacle detected! Pausing...")
            self.pause = True
            self.cancel_current_goal()
        else:
            if self.pause:
                self.get_logger().info("✅ Path clear. Retrying goal...")
                self.pause = False
                if self.current_goal_id is not None:
                    self.send_goal(self.current_goal_id)

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.get_logger().error("🛑 EMERGENCY STOP!")
            self.cancel_current_goal()
            self.goal_queue.clear()
            self.sequence_running = False

    def goal_sequence_callback(self, msg):
        if msg.data not in self.sequences:
            self.get_logger().warning(f"❓ Unknown sequence: {msg.data}")
            return
        self.cancel_current_goal()
        self.goal_queue = self.sequences[msg.data][:]
        self.get_logger().info(f"📌 Sequence {msg.data} starting.")
        self.sequence_running = True
        self.schedule_next_goal()

    # ---------------------- Action Handling ----------------------
    def send_goal(self, goal_id):
        if self.low_battery or self.pause: return

        pose = self.goals.get(goal_id)
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server unavailable!')
            return

        self.current_goal_id = goal_id
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"🚀 Sending Goal ID={goal_id}")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by server')
            return
        self._goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # FIXED FOR HUMBLE: Check status, not result.error_code
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Goal reached!")
            if self.goal_queue:
                self.schedule_next_goal()
            else:
                self.get_logger().info("🎯 All goals in sequence finished.")
                self.sequence_running = False
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("🚫 Goal was canceled (Pause/Obstacle).")
        else:
            self.get_logger().warning(f"⚠️ Goal ended with status: {status}")

    def cancel_current_goal(self):
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()

    def schedule_next_goal(self):
        if self.timer: self.timer.cancel()
        self.get_logger().info(f"⏳ Waiting {self.delay_between_goals}s...")
        self.timer = self.create_timer(self.delay_between_goals, self._timer_callback)

    def _timer_callback(self):
        self.timer.cancel()
        if self.goal_queue:
            next_id = self.goal_queue.pop(0)
            self.send_goal(next_id)

    def resume_sequence_after_battery(self):
        if self.resume_delay_timer: self.resume_delay_timer.cancel()
        if self.current_goal_id is not None:
            self.send_goal(self.current_goal_id)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()