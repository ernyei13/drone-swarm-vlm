import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(Twist, '/model/x500_quadcopter/cmd_vel', 10)
        self.get_logger().info('Keyboard control node started')

        self.twist_msg = Twist()
        self.speed = 2.0
        self.turn = 1.0

        self.key_map = {
            'w': (self.speed, 0.0, 0.0, 0.0),
            's': (-self.speed, 0.0, 0.0, 0.0),
            'a': (0.0, self.speed, 0.0, 0.0),
            'd': (0.0, -self.speed, 0.0, 0.0),
            'q': (0.0, 0.0, self.speed, 0.0),
            'e': (0.0, 0.0, -self.speed, 0.0),
            'z': (0.0, 0.0, 0.0, self.turn),
            'c': (0.0, 0.0, 0.0, -self.turn),
        }

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            char_key = key.char
            if char_key in self.key_map:
                self.update_twist(self.key_map[char_key])
        except AttributeError:
            pass

    def on_release(self, key):
        self.update_twist((0.0, 0.0, 0.0, 0.0))
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def update_twist(self, values):
        self.twist_msg.linear.x = values[0]
        self.twist_msg.linear.y = values[1]
        self.twist_msg.linear.z = values[2]
        self.twist_msg.angular.z = values[3]
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    keyboard_control_node = KeyboardControlNode()
    rclpy.spin(keyboard_control_node)
    keyboard_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
