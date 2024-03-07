import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from playground_interfaces.msg import StaticFloat32ArrayLarge
from time import perf_counter


class HeavyPubNode(Node):
    """Node which publishes significant data."""

    def __init__(self):
        super().__init__('HeavyPubNode')
        self.publisher_ = self.create_publisher(Image, 'heavy/image1000', 10)
        self.publisher_static_float = self.create_publisher(
            StaticFloat32ArrayLarge, 'heavy/static', 10)
        # Declare the 'fps' parameter with a default value
        self.declare_parameter('fps', 10)
        # Get the 'fps' parameter value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        # Ensure fps is not zero to avoid division by zero error
        fps = max(fps, 1)

        timer_period = 1/fps  # seconds (10Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.br = CvBridge()

    def timer_callback(self):
        # Generate a 480x640 random noise image
        noise_img = np.random.randint(0, 256, (1000, 1000, 3), dtype=np.uint8)

        # Convert the numpy array image to ROS Image message
        ros_img = self.br.cv2_to_imgmsg(noise_img, encoding="bgr8")

        # Publish the image
        now = perf_counter()
        self.publisher_.publish(ros_img)
        self.get_logger().info(
            f'Publishing random noise image at {perf_counter()-now:.6f} seconds')

        msg = StaticFloat32ArrayLarge()
        msg.data = np.random.randn(100000).astype(np.float32)
        now = perf_counter()
        self.publisher_static_float.publish(msg)
        self.get_logger().info(
            f'Publishing random static floats at {perf_counter()-now:.6f} seconds')


def main(args=None):
    rclpy.init(args=args)
    node = HeavyPubNode()
    try:
        # Spin the node so it can perform its work (e.g., handling subscriptions, timers)
        rclpy.spin(node)
    except KeyboardInterrupt as ki:
        # Destroy the node explicitly - should do this in every Exception type except for
        # BaseException.
        node.destroy_node()
    finally:
        print('Node has been shut down gracefully')


if __name__ == '__main__':
    main()
