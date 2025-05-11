
# TODO: Replace all uses of <ClassName> with the name of your class
# TODO: Replace all uses of <ServiceType> with the name of the service message you just made in rocko_interfaces

from rocko_interfaces.srv import <ServiceType>

class <ClassName>(Node):

    def __init__(self):
        super().__init__('<ClassName>_node')
        # Handle any hardware initialization here

        # Create a new service to send data to ros2_control
        
        self.srv = self.create_service(<ServiceType>, '<ClassName>_service', self.callback)


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