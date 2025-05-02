from flask import Flask, jsonify, render_template
import threading
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

# Placeholder for ROS imports and setup
# import rospy
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32

app = Flask(__name__)

latest_data = {
    "leftmotorpwm": 0,
    "rightmotorpwm": 0,
}

def ros_listener():
    """
    Listen to ros topics. 
    subscribe to the leftmotorcontroller/pwm and rightmotorcontroller/pwm topics.
    add the lastest  data to the latest_data dictionary."""
    def left_motor_callback(msg):
        latest_data["leftmotorpwm"] = msg.data

    def right_motor_callback(msg):
        latest_data["rightmotorpwm"] = msg.data

    rospy.Subscriber('/leftmotorcontroller/pwm', Float32, left_motor_callback)
    rospy.Subscriber('/rightmotorcontroller/pwm', Float32, right_motor_callback)
    rospy.init_node('webserver_listener', anonymous=True)
    rospy.spin()


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
    app.run(debug=True)
