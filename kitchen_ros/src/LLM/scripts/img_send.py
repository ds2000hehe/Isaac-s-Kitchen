#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zmq
import cv2


class ImageSender(Node):
    def __init__(self):
        super().__init__('zmq_image_sender')

        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind('tcp://*:5555')
        self.get_logger().info('ZMQ ImageSender initialized and bound to tcp://*:5555')

        self.img_sub = self.create_subscription(Image, '/rgb', self.cbfunc, 10)

        self.bridge = CvBridge()

    def cbfunc(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Encode image as JPEG
            _, buffer = cv2.imencode('.jpg', frame)

            # Send with topic prefix "frame "
            self.socket.send(b"frame " + buffer.tobytes())

        except Exception as e:
            self.get_logger().error(f'didnt work: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()