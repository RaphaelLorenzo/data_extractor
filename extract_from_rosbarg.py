import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo


class MinimalSubscriber(Node):

    def __init__(self):
        print("Intializing MinimalSubscriber")
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.header.frame_id)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()