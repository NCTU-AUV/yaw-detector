#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_image)  # Publish one image per second
        self.cv_bridge = CvBridge()
        self.image_path = "src/yaw_detector/image_publisher/images"  # Assuming your images are in the "images" directory

    def publish_image(self):
        image_files = [
            "tile_pos90.png", "tile_pos75.png", "tile_pos45.png", "tile_pos30.png", "tile_pos15.png",
            "tile_0.png", "tile_neg15.png", "tile_neg30.png", "tile_neg45.png", "tile_neg75.png", "tile_neg90.png"
        ]

        for image_file in image_files:
            image_path = os.path.join(os.getcwd(), self.image_path, image_file)

            if os.path.isfile(image_path):
                image = cv2.imread(image_path)
                image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
                self.publisher.publish(image_msg)
                self.get_logger().info(f'Published image: {image_file}')
                time.sleep(0.1)  # Wait for 1 second

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
