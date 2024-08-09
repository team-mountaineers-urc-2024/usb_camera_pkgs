import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from theora_image_transport.msg import Packet
from time import sleep

class TheoraMux(Node):

    def __init__(self):
        super().__init__('theora_mux')

        # Declare Parameters
        self.declare_parameter("theora_input", "/logitech_05/theora")
        self.declare_parameter("theora_output", "/logitech_05/image_raw/theora")
        self.declare_parameter("theora_service", "/logitech_05/recall_header")
        self.declare_parameter("reset_period", 1.0)

        self.theora_out = self.create_publisher(
            Packet,
            self.get_parameter("theora_output").get_parameter_value().string_value,
            qos_profile_sensor_data
        )
        
        self.theora_in = self.create_subscription(
            Packet,
            self.get_parameter("theora_input").get_parameter_value().string_value,
            self.in_callback,
            qos_profile_sensor_data
        )

        # self.theora_republisher = self.create_timer(
        #     self.get_parameter("reset_period").get_parameter_value().double_value,
        #     self.timer_callback
        # )

        self.header_service = self.create_service(
            Trigger,
            self.get_parameter("theora_service").get_parameter_value().string_value,
            self.header_callback
        )

        self.header = [Packet] * 3
        self.header_index = 0
        self.publish_index = 0
        self.theora_lock = False

    def in_callback(self, theora_packet):

        # If the message is part of the header, save it
        if (self.header_index < 3):
            self.get_logger().info(f"Saving message {self.header_index + 1} of 3")
            self.header[self.header_index] = theora_packet
            self.header_index += 1

        # If the header messages are currently being published, don't forwards these messages
        if (self.theora_lock):
            return
        
        # Otherwise, republish the message
        self.theora_out.publish(theora_packet)
    
    def header_callback(self, request, response):

        # If the header index is to small, say so
        if self.header_index != 3:
            self.get_logger().warn(f"Header Index is {self.header_index} instead of 3. Info may not be passed properly")
            response.message = f"Header Index is {self.header_index} instead of 3. Info may not be passed properly"

        # Otherwise set the lock to publish the header
        self.get_logger().info(f"Disabling passthrough mode, sending header information")
        self.theora_lock = True
        
        for packet in self.header:
            sleep(0.1)
            self.theora_out.publish(packet)

        self.theora_lock = False
        # self.publish_index = 0
        response.success = True
        return response

    def timer_callback(self):

        # Short circuit if we don't want the header
        if not self.theora_lock:
            return
        
        # If we have fallen off the list, we're done
        if (self.publish_index > 2):
            self.get_logger().info(f"Header Complete, returning to previous passthrough mode")
            self.theora_lock = False
            return

        self.theora_out.publish(self.header[self.publish_index])
        self.get_logger().info(f"Publishing message {self.publish_index + 1} of 3")
        self.publish_index += 1
        

def main(args=None):
    rclpy.init(args=args)

    theora_mux = TheoraMux()

    rclpy.spin(theora_mux)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    theora_mux.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
