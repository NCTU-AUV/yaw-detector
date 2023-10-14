#!/usr/bin/env python3

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32  # Import Float32 message type

class YawAngleCalculator(Node):
    def __init__(self):
        super().__init__('yaw_angle_calculator')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',  # Replace with your image topic
            self.image_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.publisher = self.create_publisher(Float32, 'yaw_angle', 10)  # Publish yaw angle as Float32

    def image_callback(self, msg):
        start = time.process_time()
        image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        yaw_angle = self.calculate_yaw_angle(image)
        self.get_logger().info(f'Yaw Angle: {yaw_angle}')
        yaw_angle_msg = Float32()
        yaw_angle_msg.data = yaw_angle
        self.publisher.publish(yaw_angle_msg)

        dt = time.process_time() - start
        print(f'Execution Time: {dt * 1000}ms')

    def calculate_yaw_angle(self, image):
        # Your yaw angle calculation code here
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grayscale, 250, 300, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

        data = lines[:, 0, :]
        mean_theta = np.mean(data[:, 1])

        smaller_data = data[data[:, 1] < mean_theta]
        greater_data = data[data[:, 1] >= mean_theta]

        smaller_data = smaller_data[np.argsort(smaller_data[:, 0])]
        greater_data = greater_data[np.argsort(greater_data[:, 0])]

        smaller_step = np.mean(np.diff(smaller_data[:, 0]))
        greater_step = np.mean(np.diff(greater_data[:, 0]))

        selected_data = smaller_data if smaller_step > greater_step else greater_data

        return np.rad2deg(np.mean(selected_data[:, 1])) - 90

def main(args=None):
    rclpy.init(args=args)
    node = YawAngleCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
