# TODO: Replace all uses of <ClassName> with the name of your class
class <ClassName>(Node):

    def __init__(self):
        # Handle any hardware initialization here

        # Create a new service to send data to ros2_control
        super().__init__('node')
        self.srv = self.create_service(<ServiceType>, 'serviceName', self.callback)


    def callback(self, request, response):
        # Interact with hardware here based on info in request (if there is any)

        # Put data into response according to the service type declared in init

        return response

def main(args=None):
    rclpy.init(args=args)

    node = <ClassName>()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
