import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import CustomMsg                            # CHANGE
from sensor_msgs.msg import PointCloud2

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Declare variables to store PointCloud2 message data
        self.header, self.height, self.width, self.fields, self.is_bigendian = None, None, None, None, None
        self.point_step, self.row_step, self.data, self.is_dense = None, None, None, None

        # Publisher for CustomMsg
        self.publisher_ = self.create_publisher(CustomMsg, 'topic', 10)  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Subscribe to PointCloud2 messages just like in MinimalSubscriber.py
        self.subscription = self.create_subscription(
            PointCloud2,                                               # CHANGE
            '/cf_0/lidar/points',
            self.listener_callback,
            10)
        self.subscription

    def timer_callback(self):
        msg = CustomMsg()  
        msg.header = self.header
        msg.timebase = 0 # placeholder
        msg.point_num = 0 # placeholder
        msg.lidar_id = 0 # placeholder
        msg.rsvd = [0, 0, 0] # placeholder
        msg.points = [0, 0, 0, 0, 0, 0, 0] # placeholder




        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.point_num)       # CHANGE
        self.i += 1

    def listener_callback(self, msg):
        # Store the received PointCloud2 message data
        self.header = msg.header
        self.height = msg.height
        self.width = msg.width
        self.fields = msg.fields
        self.is_bigendian = msg.is_bigendian
        self.point_step = msg.point_step
        self.row_step = msg.row_step
        self.data = msg.data
        self.is_dense = msg.is_dense

        # Just log them
        self.get_logger().info(f'\n\nI heard header: {self.header}')
        self.get_logger().info(f'I heard height: {self.height}')
        self.get_logger().info(f'I heard width: {self.width}')
        self.get_logger().info(f'I heard fields: {self.fields}')
        self.get_logger().info(f'I heard is_bigendian: {self.is_bigendian}')
        self.get_logger().info(f'I heard point_step: {self.point_step}')
        self.get_logger().info(f'I heard row_step: {self.row_step}')
        self.get_logger().info(f'I heard data: {self.data}')
        self.get_logger().info(f'I heard is_dense: {self.is_dense}')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()