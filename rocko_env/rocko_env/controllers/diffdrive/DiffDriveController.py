#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist, Vector3
from control_msgs.msg import MultiDOFCommand
from rocko_env.controllers.utils.RateLimiter import RateLimiter
from std_msgs.msg import Float64
import math

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')
        # Set up publishers to command pid controllers
        self.left_controller_topic = self.create_publisher(MultiDOFCommand, 'left_velocity_pid_controller/reference', 10)
        self.right_controller_topic = self.create_publisher(MultiDOFCommand, 'right_velocity_pid_controller/reference', 10)
        # Publishers for feedforward
        self.left_feedforward_topic = self.create_publisher(Float64, 'left_feedforward', 10)
        self.right_feedforward_topic = self.create_publisher(Float64, 'right_feedforward', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.command_controller)
        
        # Set up subscriber to get robot body vector
        self.robot_body_vector_updated_topic = self.create_subscription(
            Twist,
            'robot_body_vector',
            self.robot_body_vector_updated,
            10)
        
        # Set up variables to hold data
        self.desired_robot_body_vector = Twist()
        
        # Robot character numbers
        self.wheel_radius_meters = 0.072
        
        # Configure RateLimiter
        # TODO: all of these are completely arbitrary rn. use foxglove to moderate
        # 1st derivative limits should represent the "acceleration limit"
        # 2nd derivative limits should represent the limit for "jerk" (del a/ del t)
        # min/max values should be whatever our intended min and max values are
        self.left_limiter = RateLimiter(
            min_value=-100, max_value=100,
            min_first_derivative_neg=-5, max_first_derivative_pos=10,
            min_second_derivative=-50, max_second_derivative=50            
        )
        self.right_limiter = RateLimiter(
            min_value=-100, max_value=100,
            min_first_derivative_neg=-3, max_first_derivative_pos=3,
            min_second_derivative=-50, max_second_derivative=50
        )
        
        # Set up variables for holding data
        self.left_vel = 0
        self.right_vel = 0
        self.left_v0 = 0
        self.left_v1 = 0
        self.right_v0 = 0
        self.right_v1 = 0
        self.init = False # error-proofing for not importing an automatic 0
        self.wheel_distance = 0.356 # meters
        
    def command_controller(self):
        if not self.init:
            return

        TURN_GAIN = 15.0
        ANG_BOOST = 0.2
        SPIN_ACCEL = 4.0
        STRAIGHT_ACCEL = 2.0
        MAX_WHEEL_VEL = 15.0

        linear_vel  = self.desired_robot_body_vector.linear.x
        angular_vel = self.desired_robot_body_vector.angular.z * TURN_GAIN

        left_cmd  = linear_vel - angular_vel * self.wheel_distance / 2.0
        right_cmd = linear_vel + angular_vel * self.wheel_distance / 2.0

        if left_cmd * right_cmd < 0:
            left_cmd  += ANG_BOOST * math.copysign(1, left_cmd)
            right_cmd += ANG_BOOST * math.copysign(1, right_cmd)

        accel = SPIN_ACCEL if abs(angular_vel) > 0.2 else STRAIGHT_ACCEL
        self.left_limiter.first_deriv_pos  =  accel
        self.left_limiter.first_deriv_neg  = -accel
        self.right_limiter.first_deriv_pos =  accel
        self.right_limiter.first_deriv_neg = -accel

        left_cmd  = max(-MAX_WHEEL_VEL,  min(MAX_WHEEL_VEL,  left_cmd))
        right_cmd = max(-MAX_WHEEL_VEL,  min(MAX_WHEEL_VEL, right_cmd))

        smooth_left  = self.left_limiter.limit(left_cmd , self.left_v0 , self.left_v1 , self.timer_period)
        smooth_right = self.right_limiter.limit(right_cmd, self.right_v0, self.right_v1, self.timer_period)

        self.left_v1,  self.left_v0  = self.left_v0,  smooth_left
        self.right_v1, self.right_v0 = self.right_v0, smooth_right

        left_msg = MultiDOFCommand()
        left_msg.dof_names  = ["left_wheel_joint"]
        left_msg.values     = [smooth_left]
        self.left_controller_topic.publish(left_msg)

        right_msg = MultiDOFCommand()
        right_msg.dof_names = ["right_wheel_joint"]
        right_msg.values    = [smooth_right]
        self.right_controller_topic.publish(right_msg)

        self.left_feedforward_topic.publish(Float64(data=smooth_left  / self.wheel_radius_meters))
        self.right_feedforward_topic.publish(Float64(data=smooth_right / self.wheel_radius_meters))
        
    def robot_body_vector_updated(self, msg):
        # Store desired robot body vector
        self.desired_robot_body_vector = msg.twist
        self.init = True

def main(args=None):
    rclpy.init(args=args)

    diff_drive_controller = DiffDriveController()

    rclpy.spin(diff_drive_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    diff_drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()