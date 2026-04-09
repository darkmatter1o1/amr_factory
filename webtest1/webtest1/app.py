from flask import Flask, render_template, jsonify, request, Response
from threading import Thread
import rclpy
from ros_thread import AMRInterface, battery_level_data, robot_position, latest_depth_frame, depth_lock
import time
import subprocess
from rclpy.task import Future
from std_srvs.srv import Empty




app = Flask(__name__)
ros_node = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/battery')
def get_battery():
    return jsonify(battery_level=battery_level_data["level"])

@app.route('/robot_position')
def get_robot_position():
    return jsonify(ros_node.get_position())

@app.route("/robot_path")
def robot_path():
    return jsonify(ros_node.get_path())

@app.route('/send_goal', methods=['POST'])
def send_goal():
    goal = request.json.get("goal")
    ros_node.publish_goal(int(goal))
    return jsonify({"status": "success", "sent_goal": goal})

@app.route('/send_twist', methods=['POST'])
def send_twist():
    data = request.get_json()
    linear = float(data.get("linear", 0.0))
    angular = float(data.get("angular", 0.0))
    ros_node.publish_twist(linear, angular)
    return jsonify({"status": "success"})

@app.route('/depth_feed')
def depth_feed():
    def generate_depth_frames():
        import ros_thread
        while True:
            with ros_thread.depth_lock:
                frame = ros_thread.latest_depth_frame
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.05)
            
    return Response(generate_depth_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/topic_health")
def topic_health():
    if ros_node is None:
        return jsonify({
            "robot": False,
            "map": False,
            "navigation": False,
            "lidar": False
        })

    return jsonify(ros_node.get_topic_health())

@app.route("/creator_backdoor")
def creator_backdoor():
    return render_template("creator_backdoor.html")

@app.route("/all_topics")
def all_topics():
    if ros_node is None:
        return jsonify([])
    return jsonify(ros_node.get_all_topics())

@app.route("/system_state")
def system_state():
    if ros_node is None:
        return jsonify({"emergency_active": False})
    return jsonify(ros_node.get_system_state())

@app.route("/safety_status")
def safety_status():
    if ros_node is None:
        return jsonify({"joystick_override": False, "obstacle_detected": False, "obstacle_duration": 0})
    return jsonify(ros_node.get_safety_status())



@app.route("/restart", methods=["POST"])
def restart():
    try:
        print("Attempting to reboot the system...")
        subprocess.Popen(["sudo", "reboot"])
        return jsonify({"status": "System restarting..."})
    except Exception as e:
        print(f"Restart error: {e}")
        return jsonify({"status": f"Failed to restart: {str(e)}"}), 500

@app.route("/shutdown", methods=["POST"])
def shutdown():
    try:
        print("Attempting to shutdown the system...")
        subprocess.Popen(["sudo", "shutdown", "-h", "now"])
        return jsonify({"status": "System shutting down..."})
    except Exception as e:
        print(f"Shutdown error: {e}")
        return jsonify({"status": f"Failed to shutdown: {str(e)}"}), 500

@app.route("/send_initial_pose", methods=["POST"])
def send_initial_pose():
    try:
        data = request.get_json()
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        yaw_deg = float(data.get("yaw_deg", 0.0))

        sent_pose = ros_node.publish_initial_pose(x, y, yaw_deg)
        return jsonify({"status": "success", "sent_pose": sent_pose})
    except Exception as e:
        return jsonify({"status": "failed", "message": str(e)}), 500

@app.route("/emergency_stop", methods=["POST"])
def emergency_stop():
    try:
        ros_node.activate_emergency()
        return jsonify({"status": "Emergency activated: continuously publishing True"})
    except Exception as e:
        print(f"[ERROR] Emergency stop failed: {e}")
        return jsonify({"status": f"Failed to trigger emergency: {str(e)}"}), 500

@app.route("/cancel_goal", methods=["POST"])
def cancel_goal():
    try:
        ros_node.cancel_current_goal()
        return jsonify({"status": "Goal cancelled"})
    except Exception as e:
        print(f"[ERROR] Cancel goal failed: {e}")
        return jsonify({"status": f"Failed to cancel goal: {str(e)}"}), 500





def ros_spin():
    global ros_node
    rclpy.init()
    ros_node = AMRInterface()
    rclpy.spin(ros_node)

if __name__ == '__main__':
    Thread(target=ros_spin, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=True)
