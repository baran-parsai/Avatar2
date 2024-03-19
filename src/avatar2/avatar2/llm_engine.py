import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import TaggedString
from rclpy.qos import QoSProfile
import os

class LLM:
    def __init__(self):
        print("LLM class created")

    def response(self, s):
        return "My response is " + str(s)

class LLMEngine(Node):
    def __init__(self):
        super().__init__('llm_engine_node')
        self.declare_parameter('out_topic', '/avatar2/out_message')
        outTopic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.declare_parameter('in_topic', '/avatar2/in_message')
        inTopic = self.get_parameter('in_topic').get_parameter_value().string_value

        self._llm = LLM()


        self.create_subscription(TaggedString, inTopic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, outTopic, QoSProfile(depth=1))


    def _callback(self, msg):
        """Deal with translation"""
        self.get_logger().info(f"{self.get_name()} listening got {msg.text.data}")

        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = msg.audio_sequence_number
        tagged_string.text.data = self._llm.response(msg.text.data)
        self._publisher.publish(tagged_string)

def main(args=None):
    rclpy.init(args=args)
    node = LLMEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()
