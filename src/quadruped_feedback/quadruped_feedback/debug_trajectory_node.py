import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import python_st3215 #
import math
import time

# Parametry z Twojej konfiguracji
THIGH_LENGTH = 0.09
SHIN_LENGTH = 0.095
GROUND_Y = -0.13
STEP_LEN = 0.08
STEP_HEIGHT = 0.05
LOOP_RATE = 40.0
DT = 1.0 / LOOP_RATE
STEP_DURATION = 1.0


class DebugTrajectoryNode(Node):
    def __init__(self):
        super().__init__('debug_trajectory_node')
        
        # 1. Inicjalizacja magistrali
        try:
            self.bus = python_st3215.ST3215(port='/dev/ttyACM0', baudrate=1000000, read_timeout=0.01)
            self.get_logger().info("Połączono z serwami ST3215")
        except Exception as e:
            self.get_logger().error(f"Błąd portu: {e}")
            return

        # 2. Publisher dla zintegrowanej wiadomości [target_h, target_k, real_h, real_k]
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_debug_data', 10)
        
        # 3. Timer sterujący pętlą
        self.timer = self.create_timer(DT, self.control_loop)
        
        self.time_elapsed = 0.0
        self.last_targets = [2048.0, 2048.0] # Przechowujemy stopnie dla porównania

        self.prev_h = 0.0
        self.prev_k = 0.0
        self.time_elapsed = 0.0
        self.timer = self.create_timer(DT, self.control_loop) #

    def solve_ik(self, x, y):
        """Kinematyka odwrotna"""
        D_sq = x**2 + y**2
        D = math.sqrt(D_sq)
        if D > (THIGH_LENGTH + SHIN_LENGTH): return None
        
        beta = math.acos(max(-1.0, min(1.0, (THIGH_LENGTH**2 + SHIN_LENGTH**2 - D_sq) / (2 * THIGH_LENGTH * SHIN_LENGTH))))
        knee_deg = 180.0 - math.degrees(beta) #
        
        phi = math.atan2(y, -x)
        alpha = math.acos(max(-1.0, min(1.0, (D_sq + THIGH_LENGTH**2 - SHIN_LENGTH**2) / (2 * D * THIGH_LENGTH))))
        hip_deg = math.degrees(alpha) - math.degrees(-phi) + 90.0 #
        
        return hip_deg, knee_deg

    def get_d_trajectory(self, t):
        """Trajektoria litery D w 'starym stylu'"""
        duration = STEP_DURATION
        progress = (t % duration) / duration
        
        if progress < 0.5: # Faza STANCE (prosta)
            x = (STEP_LEN / 2.0) - ((progress / 0.5) * STEP_LEN)
            y = GROUND_Y
        else: # Faza SWING (łuk sinus)
            sub = (progress - 0.5) / 0.5
            x = -(STEP_LEN / 2.0) + (sub * STEP_LEN)
            y = GROUND_Y + STEP_HEIGHT * math.sin(math.pi * sub)
        return x, y

    def control_loop(self):
        # --- KROK 1: ODCZYT FEEDBACKU (Rzeczywisty stan po poprzednim ruchu) ---
        feedback_degrees = []
        for s_id in [1, 2]:
            # Rejestr 56 to Present Position
            p = self.bus.send_instruction(s_id, 0x02, [56, 2])
            res = self.bus.read_response(p) #
            if res:
                parsed = self.bus.parse_response(res) #
                if parsed and parsed.get('checksum_valid'):
                    val = parsed['parameters'][0] | (parsed['parameters'][1] << 8)
                    # Konwersja wsteczna na stopnie
                    deg = (val - 1024.0) * 360.0 / 4096.0
                    feedback_degrees.append(deg)
                else: feedback_degrees.append(0.0)
            else: feedback_degrees.append(0.0)

        # --- KROK 2: PUBLIKACJA (Synchronizujemy STARY rozkaz z NOWYM feedbackiem) ---
        if len(feedback_degrees) == 2:
            msg = Float64MultiArray()
            # data[0,1] = ROZKAZ Z POPRZEDNIEJ PĘTLI, data[2,3] = OBECNY FEEDBACK
            msg.data = [self.prev_h, self.prev_k, feedback_degrees[0], feedback_degrees[1]]
            self.publisher_.publish(msg)

        # --- KROK 3: OBLICZENIA DLA NOWEGO CYKLU (Trajektoria D) ---
        tx, ty = self.get_d_trajectory(self.time_elapsed) #
        angles = self.solve_ik(tx, ty) #

        if angles:
            h_deg, k_deg = angles
            
            # Zapamiętujemy ten rozkaz jako "poprzedni" dla następnej pętli
            self.prev_h = h_deg
            self.prev_k = k_deg

            # Konwersja na binarne
            h_bin = int((h_deg / 360.0) * 4096.0 + 1024.0)
            k_bin = int(1024.0 + ((k_deg / 360.0) * 4096.0))
            k_bin = max(1350, min(2680, k_bin)) # Clamp kolana

            # --- KROK 4: WYSYŁANIE NOWEGO ROZKAZU ---
            # Goal Position (Rejestr 42)
            p1 = self.bus.send_instruction(1, 0x03, [42, h_bin & 0xFF, (h_bin >> 8) & 0xFF])
            self.bus.read_response(p1) #
            p2 = self.bus.send_instruction(2, 0x03, [42, k_bin & 0xFF, (k_bin >> 8) & 0xFF])
            self.bus.read_response(p2) #

        self.time_elapsed += DT #

def main(args=None):
    rclpy.init(args=args)
    node = DebugTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()