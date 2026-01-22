import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class convert_pc2_to_customMsg(Node):

    def __init__(self):
        super().__init__('convert_pc2_to_customMsg')
        self.subscription = self.create_subscription(
            String,
            '/cf_0/lidar/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    converter = convert_pc2_to_customMsg()

    rclpy.spin(converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()