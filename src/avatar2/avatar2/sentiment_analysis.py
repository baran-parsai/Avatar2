import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import AudioInput, SentimentAnalysis
from rclpy.qos import QoSProfile
from .sentimentModule import Sentiment

class SentimentAnalysisNode(Node):
    def __init__(self):
        super().__init__('audio_2_text_node')
        self.declare_parameter('topic', '/avatar2/audio_input')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('sentiment', '/avatar2/sentiment')
        sentiment = self.get_parameter('sentiment').get_parameter_value().string_value

        self._sentimentAnalysis = Sentiment()

        self.create_subscription(AudioInput, topic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(SentimentAnalysis, sentiment, QoSProfile(depth=1))

    def _callback(self, data):
        score = self._sentimentAnalysis.analyze(data.audio)

        sentiment_analysis = SentimentAnalysis()
        sentiment_analysis.header.stamp = self.get_clock().now().to_msg()
        sentiment_analysis.audio_sequence_number = data.seq
        sentiment_analysis.result = score
        self._publisher.publish(sentiment_analysis)

def main(args=None):
    rclpy.init(args=args)
    node = SentimentAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
