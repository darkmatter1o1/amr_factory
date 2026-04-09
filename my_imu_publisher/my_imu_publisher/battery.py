import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import serial
import threading
from collections import deque

class SerialBatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # --- 1. Publishers & Subscribers ---
        self.publisher_ = self.create_publisher(Float32, '/battery_level1', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # --- 2. State Tracking & Thread Safety ---
        self.serial_port = '/dev/arduinob'
        self.baud_rate = 9600
        self.is_moving = False
        self.obstacle_detected = False
        self.serial_lock = threading.Lock() # Prevents serial collisions
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to {self.serial_port}. Monitoring Movement & Obstacles.")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            self.ser = None

        # --- 3. Battery Filtering Variables ---
        self.max_voltage, self.min_voltage = 26.8, 10.2
        self.readings = deque(maxlen=20)
        self.filtered_voltage = None
        self.ema_alpha = 0.3

        # --- 4. Timers ---
        # Frequency for reading battery data
        self.timer = self.create_timer(0.1, self.timer_callback) 
        # Frequency for ensuring Arduino stays in the correct state
        self.heartbeat_timer = self.create_timer(1.0, self.send_heartbeat)

    def safe_serial_write(self, command):
        """ Thread-safe helper to write to Serial """
        if self.ser and self.ser.is_open:
            with self.serial_lock:
                self.ser.write(command)

    def publish_obstacle_status(self, state):
        """ Helper to publish boolean status to ROS topic """
        msg = Bool()
        msg.data = state
        self.obstacle_pub.publish(msg)

    def send_heartbeat(self):
        """ Periodically reminds Arduino of the current state """
        if self.obstacle_detected:
            self.safe_serial_write(b'O')
        elif self.is_moving:
            self.safe_serial_write(b'M')
        else:
            self.safe_serial_write(b'S')

    def scan_callback(self, msg):
        """ Checks for obstacles ONLY when the robot is moving """
        # If robot is stationary, we don't care about obstacles for this logic
        if not self.is_moving:
            if self.obstacle_detected: # Clean up state if we just stopped
                self.obstacle_detected = False
                self.publish_obstacle_status(False)
            return

        # Define 30-degree front window
        window = 15 
        front_sector = msg.ranges[:window] + msg.ranges[-window:]
        valid_ranges = [r for r in front_sector if msg.range_min < r < msg.range_max]
        
        # Detection Threshold: 1.0 meter
        if valid_ranges and min(valid_ranges) < 1.0:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.safe_serial_write(b'O')
                self.publish_obstacle_status(True)
                self.get_logger().warn("OBSTACLE DETECTED WHILE MOVING!")
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.safe_serial_write(b'M') # Resume "Moving" state on Arduino
                self.publish_obstacle_status(False)
                self.get_logger().info("Path Clear.")

    def cmd_vel_callback(self, msg):
        """ Monitors velocity commands to toggle monitoring state """
        # Check if linear or angular velocity is non-trivial
        currently_moving = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)
        
        if currently_moving != self.is_moving:
            self.is_moving = currently_moving
            
            if not self.is_moving:
                # Robot has come to a stop
                self.obstacle_detected = False 
                self.publish_obstacle_status(False)
                self.safe_serial_write(b'S')
                self.get_logger().info("Robot Stopped. Obstacle monitoring idle.")
            else:
                # Robot has started moving
                self.safe_serial_write(b'M')
                self.get_logger().info("Robot Moving. Obstacle monitoring active.")

    def timer_callback(self):
        """ Processes incoming battery voltage strings from Arduino """
        if not self.ser: return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if not line or ":" not in line: continue
                
                try:
                    # Expected format: "V:24.5"
                    raw_v = float(line.split(":")[1])
                except (ValueError, IndexError):
                    continue

                # Moving Average + EMA Filter
                self.readings.append(raw_v)
                avg_v = sum(self.readings) / len(self.readings)
                
                if self.filtered_voltage is None: 
                    self.filtered_voltage = avg_v
                else: 
                    self.filtered_voltage = (self.ema_alpha * avg_v + 
                                            (1 - self.ema_alpha) * self.filtered_voltage)

                # Clamp and calculate percentage
                stable_v = max(min(self.filtered_voltage, self.max_voltage), self.min_voltage)
                pct = round(((stable_v - self.min_voltage) / 
                             (self.max_voltage - self.min_voltage)) * 100, 2)

                msg = Float32()
                msg.data = float(pct)
                self.publisher_.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Try to send Stop command before shutting down
        node.safe_serial_write(b'S')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()