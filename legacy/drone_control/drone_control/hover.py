import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HoverNode(Node):
    def __init__(self):
        super().__init__('hover_node')
        self.publisher_ = self.create_publisher(Twist, '/model/x500_quadcopter/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_hover_command)
        self.get_logger().info('Hover node started, publishing to /model/x500_quadcopter/cmd_vel')

    def publish_hover_command(self):
        msg = Twist()
        msg.linear.z = 9.81  # Counteract gravity
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    hover_node = HoverNode()
    rclpy.spin(hover_node)
    hover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
