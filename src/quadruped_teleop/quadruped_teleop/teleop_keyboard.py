import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# initial terminal settings
settings = termios.tcgetattr(sys.stdin) 

key_bindings = {
    'w': (1.0, 0.0),  # move backwards
    's': (-1.0, 0.0), # move forward
    'a': (0.0, 1.0),  # rotate left
    'd': (0.0, -1.0), # rotate right
    'c': (0.0, 0.0)   # stop
}

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # /cmd_vel publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Teleop ready!")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # 0.05s breaks between reading the keyboard
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        last_key_time = time.time()
        
        # if there's no key pressed for more than STOP_TIMEOUT -> stop
        STOP_TIMEOUT = 0.15

        try:
            while rclpy.ok():
                key = self.get_key()
                current_time = time.time()
                
                # if a proper key is pressed:
                if key in key_bindings:
                    linear = key_bindings[key][0]
                    angular = key_bindings[key][1]
                    
                    twist = Twist()
                    twist.linear.x = float(linear)
                    twist.angular.z = float(angular)

                    # publish the command
                    self.publisher_.publish(twist)
                    
                    # updating the time of the latest command
                    last_key_time = current_time
                    
                    # printing commands
                    self.get_logger().info(f"\rCMD: {key} | Lin: {linear:.2f} | Ang: {angular:.2f}   ", end="")

                elif key == '\x03': # Ctrl+C
                    break
                
                else:
                    # if no key or invalid key is pressed, check timeout
                    if (current_time - last_key_time) > STOP_TIMEOUT:
                        
                        # published command stops the robot
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # always send STOP when the node is closed
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

            # restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("\nTeleop closed.")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()