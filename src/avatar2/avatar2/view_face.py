import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from avatar2_interfaces.msg import SpeakerInfo
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

class ViewCamera(Node):
    def __init__(self):
        super().__init__('view_face')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('face', "/avatar2/speaker_info")

        self._face_topic = self.get_parameter('face').get_parameter_value().string_value
        self.create_subscription(SpeakerInfo, self._face_topic, self._face_callback, QoSProfile(depth=1))
        self._bridge = CvBridge()

    def _face_callback(self, msg):
        self.get_logger().info(f'{self.get_name()} recovered face {msg.row} {msg.col}')
        face = self._bridge.imgmsg_to_cv2(msg.face, "bgr8")
   
        cv2.imshow('face', face)
        cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    node = ViewCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

