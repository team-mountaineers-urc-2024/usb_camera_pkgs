import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image

from rclpy.qos import qos_profile_sensor_data


class ImageFlip(Node):

    def __init__(self):
        super().__init__('image_flip')

        # image input
        self.subscription = self.create_subscription(
            Image,
            'input',
            self.image_input,
            qos_profile_sensor_data)
        
        # image output
        self.publisher = self.create_publisher(
            Image,
            'output',
            10
        )


    def image_input(self, msg):

        channels = 3

        np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))
        rotated_arr = np.flipud(np.fliplr(np_arr.reshape((msg.height, msg.width, channels))))
        msg.data = rotated_arr.tobytes()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    image_flip = ImageFlip()

    rclpy.spin(image_flip)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_flip.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()