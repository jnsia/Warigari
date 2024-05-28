import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import time

class WebInteractionNode(Node):
    def __init__(self):
        super().__init__('web_interaction_node')
        # Publisher setup
        self.publisher = self.create_publisher(String, '/chatter', 10)
        self.timer = self.create_timer(2, self.publish_message)  # 2초 간격으로 메시지 발행

        # Subscriber setup
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.receive_message,
            10
        )
        self.subscription  # prevent unused variable warning

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from ROS 2 to web!'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def receive_message(self, msg):
        self.get_logger().info('Received message from web: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    web_interaction_node = WebInteractionNode()
    
    # Use a MultiThreadedExecutor to handle callbacks for both subscriber and publisher
    executor = MultiThreadedExecutor()
    executor.add_node(web_interaction_node)
    
    try:
        executor.spin()
    finally:
        web_interaction_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
