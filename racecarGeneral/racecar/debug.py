#!/usr/bin/env python3
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped


# Make a class to publish ackermannstamped messages

class DrivePub:
    def __init__(self):
        self.node = rclpy.create_node('drive_pub')
        self.pub = self.node.create_publisher(AckermannDriveStamped, '/vesc/low_level/ackermann_cmd_mux/output', 10)
        self.freq = 10
        self.count = 0
        self.period = 3  # 3 seconds
        self.on = True  # start driving

        self.timer = self.node.create_timer(1 / self.freq, self.timer_callback)

    def timer_callback(self):
        # Oscillate between driving and not driving
        if self.count < self.period * self.freq:
            self.count += 1
        else:
            self.on = not self.on
            self.count = 0

        self.node.get_logger().info(str(self.count))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.2 * self.on
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    drive_pub = DrivePub()
    rclpy.spin(drive_pub.node)
    drive_pub.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

