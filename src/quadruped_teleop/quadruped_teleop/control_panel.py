import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt6.QtWidgets import (QApplication, 
                             QWidget, 
                             QVBoxLayout, 
                             QSlider, 
                             QLabel, 
                             QGroupBox)
from PyQt6.QtCore import (Qt, 
                          QTimer)

class RobotControlPanel(Node):
    def __init__(self):
        super().__init__('python_gui_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/gait_params', 10)

    def send_gait_command(self, length, height, rotation):
        msg = Float64MultiArray()
        msg.data = [float(length), float(height), float(rotation)]
        self.publisher_.publish(msg)

class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setup_ui()
        
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(100) 

    def setup_ui(self):
        self.setWindowTitle("Robot Dog Control Center")
        self.resize(300, 400)
        
        # --- STYLESHEET (Poprawiony) ---
        self.setStyleSheet("""
            QWidget {
                background-color: #1f1f1f;
                color: white;
                font-family: 'Roboto', 'Segoe UI', sans-serif;
                font-size: 14px;
            }
            
            QGroupBox {
                background-color: #2b2b2b;
                border: 1px solid #555;     /* Nieco delikatniejsza ramka */
                border-radius: 8px;
                margin-top: 20px;           /* Miejsce na tytuł */
                font-weight: bold;
            }
            
            /* --- POPRAWKA 1: WCIĘCIE TYTUŁU --- */
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 15px;                 /* Odsunięcie od lewej krawędzi */
                padding: 0 5px;             /* Margines tekstu od linii */
                
                /* KLUCZOWE: Kolor tła taki sam jak OKNA (#1f1f1f), 
                   żeby zakryć linię ramki pod napisem */
                background-color: #1f1f1f; 
                color: #ddd;                /* Lekko jaśniejszy tekst tytułu */
            }

            /* --- POPRAWKA 2: USUNIĘCIE TŁA DOOKOŁA WARTOŚCI --- */
            QLabel#ValueLabel {
                background-color: #1f1f1f; /* Było #1f1f1f, co robiło kloca */
                padding: 6px 6px;                  /* Usunięty wielki padding */
                margin-bottom: 2px;
                border-radius: 8px;
                font-weight: normal;           /* Tekst wartości nieco lżejszy */
                font-size: 15px;
            }

            /* --- STYL SUWAKA (Bez zmian, jest OK) --- */
            QSlider {
                background: transparent;    /* Dla pewności */
            }
            
            QSlider::groove:horizontal {
                background: #555;
                height: 10px;
                border-radius: 5px;
            }

            QSlider::handle:horizontal {
                background: #ff0000;
                width: 26px;
                height: 16px;
                border-radius: 8px;
                margin: -3px 0; 
            }
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.setContentsMargins(20, 25, 20, 25)

        # --- SEKCJA 1: WYSOKOŚĆ ---
        group_h = QGroupBox("Step Height")
        layout_h = QVBoxLayout()
        # Dodajemy padding wewnątrz grupy, żeby elementy nie dotykały ramki
        layout_h.setContentsMargins(15, 25, 15, 15) 
        
        self.label_h = QLabel("5 cm")
        self.label_h.setObjectName("ValueLabel")
        layout_h.addWidget(self.label_h, alignment=Qt.AlignmentFlag.AlignLeft)

        self.slider_h = QSlider(Qt.Orientation.Horizontal)
        self.slider_h.setMinimum(1)
        self.slider_h.setMaximum(8)
        self.slider_h.setValue(5)
        self.slider_h.valueChanged.connect(self.update_values)
        layout_h.addWidget(self.slider_h)
        group_h.setLayout(layout_h)
        layout.addWidget(group_h)

        # --- SEKCJA 2: DŁUGOŚĆ ---
        group_l = QGroupBox("Step Length")
        layout_l = QVBoxLayout()
        layout_l.setContentsMargins(15, 25, 15, 15)

        self.label_l = QLabel("8 cm")
        self.label_l.setObjectName("ValueLabel")
        layout_l.addWidget(self.label_l, alignment=Qt.AlignmentFlag.AlignLeft)

        self.slider_l = QSlider(Qt.Orientation.Horizontal)
        self.slider_l.setMinimum(4)
        self.slider_l.setMaximum(22)
        self.slider_l.setValue(8)
        self.slider_l.valueChanged.connect(self.update_values)
        layout_l.addWidget(self.slider_l)
        group_l.setLayout(layout_l)
        layout.addWidget(group_l)

        # --- SEKCJA 3: ROTACJA ---
        group_r = QGroupBox("Rotation Stiffness")
        layout_r = QVBoxLayout()
        layout_r.setContentsMargins(15, 25, 15, 15)

        self.label_r = QLabel("40 %")
        self.label_r.setObjectName("ValueLabel")
        layout_r.addWidget(self.label_r, alignment=Qt.AlignmentFlag.AlignLeft)

        self.slider_r = QSlider(Qt.Orientation.Horizontal)
        self.slider_r.setMinimum(20)
        self.slider_r.setMaximum(80)
        self.slider_r.setValue(40)

        # valueChanged = if the value changes, connect() - do sth
        self.slider_r.valueChanged.connect(self.update_values)
        
        layout_r.addWidget(self.slider_r)
        group_r.setLayout(layout_r)
        layout.addWidget(group_r)

        self.setLayout(layout)

    # Update variables in the ROS node
    def update_values(self):
        h_cm = self.slider_h.value()
        l_cm = self.slider_l.value()
        r_perc = self.slider_r.value()
        
        h_meters = h_cm / 100.0
        l_meters = l_cm / 100.0
        r_factor = r_perc / 100.0

        self.label_h.setText(f"{h_cm} cm")
        self.label_l.setText(f"{l_cm} cm")
        self.label_r.setText(f"{r_perc} %")

        self.ros_node.send_gait_command(l_meters, h_meters, r_factor)

    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

def main():
    rclpy.init()
    node = RobotControlPanel()
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    try:
        sys.exit(app.exec())
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()