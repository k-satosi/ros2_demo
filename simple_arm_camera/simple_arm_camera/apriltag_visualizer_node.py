#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from apriltag_msgs.msg import AprilTagDetectionArray

class AprilTagVisualizer(Node):
    def __init__(self):
        super().__init__('apriltag_visualizer')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detections_callback, 10)
        self.image_pub = self.create_publisher(Image, '/image_with_tags', 10)
        self.current_image = None
        self.current_detections = []

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_and_publish_image()

    def detections_callback(self, msg):
        self.current_detections = msg.detections
        self.process_and_publish_image()

    def process_and_publish_image(self):
        if self.current_image is None:
            return

        output_image = self.current_image.copy()

        if self.current_detections:
            for detection in self.current_detections:
                corners = np.array([[corner.x, corner.y] for corner in detection.corners])
                corners = corners.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(output_image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

        image_msg = self.bridge.cv2_to_imgmsg(output_image, encoding="bgr8")
        self.image_pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
