import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist, Vector3
from rocko_interfaces.msg import Icm20948Data

class BalancingController(Node):

    def __init__(self):
        super().__init__('balancing_controller')
        # Set up publisher to command controller
        self.command_controller_topic = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.command_controller)
        
        # Set up subscriber to get velocity data
        self.vel_updated_topic = self.create_subscription(
            TwistStamped,
            'cmd_vel_out',
            self.vel_updated,
            10)
        
        # Set up subscriber to get IMU data
        self.imu_updated_topic = self.create_subscription(
            Icm20948Data,
            'icm20948_data',
            self.imu_updated,
            10)
        
        # Set up variables to hold data
        self.current_vel = Twist()
        self.current_angles = Icm20948Data()

    def command_controller(self):
        msg = TwistStamped()
        # Do calcs here for equations of motion
        
        # Set values of vectors based on equations of motion results
        linearVector = Vector3()
        linearVector.x = 0
        linearVector.y = 0
        linearVector.z = 0
        
        angularVector = Vector3()
        angularVector.x = 0
        angularVector.y = 0
        angularVector.z = 0

        twist = Twist()
        twist.linear = linearVector
        twist.angular = angularVector
        # publish the desired valocity for the robot
        msg.twist = twist
        self.command_controller_topic.publish(msg)
        
    def vel_updated(self, msg):
        # Store current velocity
        self.current_vel = msg.twist
        
    def imu_updated(self, msg):
        # Store current imu data
        self.current_angles.yaw = msg.yaw
        self.current_angles.roll = msg.roll
        self.current_angles.pitch = msg.pitch


def main(args=None):
    rclpy.init(args=args)

    balancing_controller = BalancingController()

    rclpy.spin(balancing_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    balancing_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()