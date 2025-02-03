import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class LowLevelRelay(Node):

    def __init__(self):
        super().__init__('low_level_relay')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/ackermann_cmd_mux/output', 10)

        # Create the subscriber. 
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/vesc/low_level/ackermann_cmd',
            self.listener_callback,
            10)

    def listener_callback(self, data):
        """
        Callback function.
        """
        self.get_logger().info("got data!")
        self.publisher_.publish(data)


def main(args=None):
    rclpy.init(args=args)

    relay = LowLevelRelay()

    rclpy.spin(relay)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

