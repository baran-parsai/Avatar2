import os
import rclpy
import json
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import TaggedString
from .llm import LLM
from .llm_dummy import LLMDummy
from .llm_langchain import LLMLangChain
from .llm_withfaces import LLMWithFaces
from .llm_local_cache import LocalCache

class LLMEngine(Node):
    def __init__(self, config_file='/home/walleed/Avatar2/hearing_clinic_config.json'):
        super().__init__('llm_engine_node')

        with open(config_file, 'r') as f:
            config = json.load(f)

        self._debug = config.get('debug', True)
        if self._debug:
            self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')

        outTopic = config.get('out_topic', '/avatar2/out_message')
        inTopic = config.get('in_topic', '/avatar2/in_message')
        avatar_type = config.get('avatar', 'dummy')

        if self._debug:
            self.get_logger().info(f'{self.get_name()} Firing up an avatar of type {avatar_type}')

        if avatar_type == 'dummy':
            self._llm = LLMDummy()
        elif avatar_type == 'langchain' or avatar_type == 'faces':
            root = config.get('root', './museum/')
            model = root + '/' + config.get('model', 'some.gguf')
            prompt = config.get('prompt', 'You are an AI assistant. Answer questions.')
            vectorstore = root + '/' + config.get('vectorstore', 'vectorstore.pkl')
            format = config.get('format', '\n###USER: {question}\n###ASSISTANT:')
            test_cache = root + '/' + config.get('test_cache', 'test_cache.json')

            if avatar_type == 'langchain':
                self.get_logger().info(f'{self.get_name()} Loading LLM model {model}')
                self._llm = LLMLangChain(model=model, prompt=prompt, vectorstore=vectorstore, format=format)
                self.local_cache = LocalCache(node=self, filename=test_cache, root=root)
            if avatar_type == 'faces':
                self.get_logger().info(f'{self.get_name()} Loading LLM model {model}')
                self._llm = LLMWithFaces(model=model, prompt=prompt, vectorstore=vectorstore, format=format, node=self)
            self.local_cache = LocalCache(node=self, filename=test_cache, root=root)

        else:
            if self._debug:
                self.get_logger().info(f'{self.get_name()} {avatar_type} not known, using dummy')
            self._llm = LLMDummy()

        self.get_logger().info(f"{self.get_name()} LLM {model} is active!")
        self.create_subscription(TaggedString, inTopic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, outTopic, QoSProfile(depth=1))

    def _callback(self, msg):
        """Deal with translation"""
        if self._debug:
            self.get_logger().info(f"{self.get_name()} listening got {msg.text.data}")
        if self._debug:
            self.get_logger().info(f"{self.get_name()} checking cache for {msg.text.data}")
        response, response_time = self.local_cache.get(msg.text.data)

        if response is None:
            start_time = self.get_clock().now()
            prompt, response = self._llm.response(text=msg.text.data)
            time_taken = (self.get_clock().now() - start_time).nanoseconds / 1e9
            # Add to cache
            self.local_cache.put(msg.text.data, response, time_taken)
            if self._debug:
                self.get_logger().info(f"{self.get_name()} response: {response}, {len(response)}")
        else:
            # Cache hit so update the cache with the response time
            self.local_cache._update_cache(msg.text.data, response, response_time)
            
        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = msg.audio_sequence_number
        tagged_string.text.data = str(response)
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
