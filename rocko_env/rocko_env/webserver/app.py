from flask import Flask, jsonify, render_template
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

app = Flask(__name__)

latest_data = {
    "leftmotorpwm": 0,
    "rightmotorpwm": 0,
}

class WebserverROSNode(Node):
    def __init__(self):
        super().__init__('webserver_listener')
        self.create_subscription(
            Float32,
            '/left_feedforward',
            self.left_motor_callback,
            10
        )
        self.create_subscription(
            Float32,
            '/right_feedforward',
            self.right_motor_callback,
            10
        )

    def left_motor_callback(self, msg):
        latest_data["leftmotorpwm"] = msg.data

    def right_motor_callback(self, msg):
        latest_data["rightmotorpwm"] = msg.data

def ros_listener():
    rclpy.init()
    node = WebserverROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

@app.route('/data')
def data():
    return jsonify(latest_data)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/plot')
def plot():
    return render_template('plot.html')

def start_ros_thread():
    t = threading.Thread(target=ros_listener)
    t.daemon = True
    t.start()

if __name__ == '__main__':
    start_ros_thread()
    app.run(host='0.0.0.0', debug=True)
