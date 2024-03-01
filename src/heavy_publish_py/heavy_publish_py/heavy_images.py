import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('my_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'heavy/image640', 10)
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
        noise_img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)

        # Convert the numpy array image to ROS Image message
        ros_img = self.br.cv2_to_imgmsg(noise_img, encoding="bgr8")

        # Publish the image
        self.publisher_.publish(ros_img)
        # self.get_logger().info('Publishing random noise image')


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
