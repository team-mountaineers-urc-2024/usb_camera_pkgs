import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(Image, '/image_bgra', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        
        # Convert ROS Image message to a numpy array
        yuy2_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 2))

        # Convert YUY2 to BGR
        bgra_image = cv2.cvtColor(yuy2_image, cv2.COLOR_YUV2BGRA_YUYV)
        
        # Convert the BGR image to ROS Image message
        bgra_msg = self.bridge.cv2_to_imgmsg(bgra_image, 'bgra8')

        # Publish the BGR image
        self.publisher.publish(bgra_msg)

def main(args=None):
    rclpy.init(args=args)

    image_converter = ImageConverter()

    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
