import rclpy
from rclpy.node import Node

from my_py_interface.msg import Lidar    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Lidar, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Lidar()                                           # CHANGE
        msg.left = True
        msg.right = False
        msg.front = True    
        msg.back = False               # CHANGE
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()