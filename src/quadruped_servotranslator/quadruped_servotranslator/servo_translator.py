import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
import python_st3215

PORT = '/dev/ttyACM0'
REG_GOAL_POS = 42 #
REG_PRESENT_POS = 56 #

class TranslatorNode(Node):
    def __init__(self):
        super().__init__('servotranslator_node')
        self.callback_group = ReentrantCallbackGroup()

        try:
            self.bus = python_st3215.ST3215(port=PORT, baudrate=1000000, read_timeout=0.01)
            self.get_logger().info(f"Połączono z portem {PORT}")
        except Exception as e:
            self.get_logger().error(f"Nie udało się otworzyć portu: {e}")
            self.bus = None

        self.subscriber_ = self.create_subscription(
            Float64MultiArray,
            'joint_group_command', 
            self.listener_callback, 
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info("Translator gotowy.")

    def listener_callback(self, msg):
        if self.bus is None or len(msg.data) < 8: return
        RIGHT_SIDE_LEGS = [1, 3]

        for i in range(4):
            hip_idx = i * 2
            knee_idx = i * 2 + 1
            
            hip_deg = msg.data[hip_idx]
            knee_deg = msg.data[knee_idx]
            is_right_side = (i in RIGHT_SIDE_LEGS)

            # INWERSJA: Tylko biodra po prawej stronie odwracamy, żeby szły do przodu
            if is_right_side:
                hip_deg = -hip_deg + 90.0
                # knee_deg zostawiamy dodatnie, żeby nie biło w limit 1350!

            raw_hip = self.translate_hip_logic(hip_deg)
            raw_knee = self.translate_knee_logic(knee_deg)
            
            # Twój specyficzny offset dla lewej strony (-950)
            final_hip = raw_hip - 950 if not is_right_side else raw_hip
            
            # ID biodra: 1,3,5,7 | ID kolana: 2,4,6,8
            self.send_single_command((i * 2) + 1, final_hip)
            self.send_single_command((i * 2) + 2, raw_knee)

    def send_single_command(self, servo_id, position_val):
        try:
            # Clamp ogólny [400, 3800]
            clamped_pos = int(max(400, min(3800, position_val)))
            # Clamp kolan [1350, 2680]
            if servo_id % 2 == 0:
                 clamped_pos = int(max(1350, min(2680, clamped_pos)))

            pos_low = clamped_pos & 0xFF
            pos_high = (clamped_pos >> 8) & 0xFF
            self.bus.send_instruction(servo_id, 0x03, [REG_GOAL_POS, pos_low, pos_high])
        except Exception as e:
            self.get_logger().error(f"Error ID {servo_id}: {e}")

    def translate_hip_logic(self, hip_deg):
        return int((hip_deg / 360.0) * 4096.0 + 1024.0)

    def translate_knee_logic(self, knee_deg):
        return int(1024.0 + ((knee_deg / 360.0) * 4096.0))

def main(args=None):
    rclpy.init(args=args)
    node = TranslatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()