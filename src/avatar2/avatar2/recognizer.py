import os
import sys
import rclpy
import cv2
import datetime
import numpy as np
import math
import json
from pathlib import Path
from cv_bridge import CvBridge
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from avatar2_interfaces.msg import SpeakerInfo
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from .FaceRecognizer import FaceRecognizer
from .FaceRecognizer import FaceRecognizer


class Recognizer(Node):
    DEFAULT_ROOT = "/home/baranparsai/Documents/Avatar2/scenarios/hearing_clinic/faces"
    def __init__(self):
        super().__init__('recognizer_node')
        self.get_logger().info(f'{self.get_name()} node created')


        # params
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')

        self._msg_id = 0
        self.declare_parameter('topic', '/avatar2/speaker_info')
        self._topic = self.get_parameter('topic').get_parameter_value().string_value

        self.declare_parameter("camera_topic", "/mycamera/image_raw")
        self._camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value

        self.declare_parameter("root_dir", Recognizer.DEFAULT_ROOT)
        root = self.get_parameter("root_dir").get_parameter_value().string_value
        if self._debug:
            self.get_logger().info(f'{self.get_name()} root is {root}')

        encodings = Path(os.path.join(root, "faces.pkl"))
        database = Path(os.path.join(root, "faces.json"))
    
        self._bridge = CvBridge()

        # subs
        self._sub = self.create_subscription(Image, self._camera_topic, self._camera_callback, 1) 

        # pubs
        self._publisher = self.create_publisher(SpeakerInfo, self._topic, QoSProfile(depth=1))

        self._face_recognizer = FaceRecognizer(encodings=encodings, database=database)
        self._face_recognizer.load_encodings()
        
    
    def _camera_callback(self, data):
        img = self._bridge.imgmsg_to_cv2(data)

        bb, name, middle_row, middle_col = self._face_recognizer.recognize_faces(img)
        if self._debug:
            self.get_logger().info(f'{self.get_name()} bounding_box {bb} name is {name}')

        if bb is not None:
            sub = img[bb[0]:bb[2], bb[3]:bb[1],:]
            if self._debug:
                cv2.imshow('face', sub)
                cv2.waitKey(3)


            sp = SpeakerInfo()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.seq = self._msg_id
            self._msg_id = self._msg_id + 1
            sp.row = float(middle_row) / float(img.shape[0])
            sp.col = float(middle_col) / float(img.shape[1])
            sp.width = float(sub.shape[1]) / float(img.shape[1])
            sp.height = float(sub.shape[0]) / float(img.shape[0])
            sp.face = self._bridge.cv2_to_imgmsg(img, "bgr8")
            sp.info = String()
            sp.info.data = json.dumps(name)
            self._publisher.publish(sp)
    
def main(args=None):
    rclpy.init(args=args)
    node = Recognizer()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

    

    


