#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Stałe portu i rejestrów
const char* SERIAL_PORT = "/dev/ttyACM0";
const int REG_GOAL_POS = 42;

class QuadrupedTranslatorNode : public rclcpp::Node {
public:
    QuadrupedTranslatorNode() : Node("servotranslator_node") {
        
        // Port opening, opening a serial port return an int variable 'fd_',
        // '0_RDWR' means read & write access,
        // 'O_NOCTTY' means no terminal control, prevents ctrl+c from terminating the program,
        // 'O_SYNC' means write() operations will wait until data is physically written in servos
        // '0_EXCL' means exclusive access to the port

        fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC | 0_EXCL);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open the port: %s", SERIAL_PORT);
        } else {
            setup_serial(1000000); // 1 Mbps
            RCLCPP_INFO(this->get_logger(), "Connected to the port: %s", SERIAL_PORT);
        }

        // Poprawiony typ wiadomości na Float64MultiArray
        joints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_group_command", 10, 
            std::bind(&QuadrupedTranslatorNode::listener_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "C++ servo translator has been initialized.");
    }

    ~QuadrupedTranslatorNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;   
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joints_sub_;

    void setup_serial(int baudrate) {
        struct termios tty;     // termios structure to configure the serial port
        if (tcgetattr(fd_, &tty) != 0) return; // tcgetattr 

        cfsetospeed(&tty, B1000000); // sending speed 1Mbps 
        cfsetispeed(&tty, B1000000); // receiving speed 1Mbps

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits of data in each byte, from 8N1 standard
        tty.c_iflag &= ~IGNBRK; // input flag
        tty.c_lflag = 0;        // local flag
        tty.c_oflag = 0;        // output flag
        tty.c_cc[VMIN] = 0;     // |c_cc = control characters| Minimum number of characters to read
        tty.c_cc[VTIME] = 2;    // Timeout 0.5s - go further even if no data received within 0.5s

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control 
        tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // No parity - checksum is used instead
        tty.c_cflag &= ~CSTOPB;                 // 1 stop bit - ST3215 requirement
        tcsetattr(fd_, TCSANOW, &tty);
    }

    void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 8) return;

        for (int i = 0; i < 4; i++) {
            int hip_idx = i * 2;
            int knee_idx = i * 2 + 1;

            float hip_deg = msg->data[hip_idx];
            float knee_deg = msg->data[knee_idx];
            
            bool is_right_side = (i == 1 || i == 3);

            if (is_right_side) {
                hip_deg = -hip_deg + 90.0; // right side inversion
            }

            int raw_hip = translate_hip_logic(hip_deg);
            int raw_knee = translate_knee_logic(knee_deg);

            // offset callibration for the right side
            int final_hip = (is_right_side) ? (raw_hip) : (raw_hip - 950);

            send_single_command((i * 2 + 1), final_hip);
            send_single_command((i * 2 + 2), raw_knee); 

            usleep(2000);
            read_current(1);
            usleep(2000);
        }
    }

    void send_single_command(int servo_id, int position_val) {
        // command clamping
        // hip
        int clamped_pos = std::max(400, std::min(3800, position_val));
        
        // knee
        if (servo_id % 2 == 0) {
            clamped_pos = std::max(1350, std::min(2680, clamped_pos));
        }
    
        unsigned char pos_low = clamped_pos & 0xFF;
        // & - bit AND operator, extracts first 8 bits (0xFF = 11111111 in binary)

        unsigned char pos_high = (clamped_pos >> 8) & 0xFF;
        // >> - bitwise right shift operator, shifts bits to the right by 8 positions


        // PACKET CREATION
        unsigned char packet[9];
        packet[0] = 0xFF; // Header 1
        packet[1] = 0xFF; // Header 2
        // Headers - 'Wake up, servo!', 2 headers are required to avoid errors
        packet[2] = (unsigned char)servo_id;
        packet[3] = 0x05; // Length: Instruction (1) + Parameters (3) + Checksum (1)
        packet[4] = 0x03; // WRITE Instruction
        packet[5] = (unsigned char)REG_GOAL_POS; // Goal Position Register
        packet[6] = pos_low;
        packet[7] = pos_high;
        // pos_low & pos_high - desired position cut into 2 bytes, 
        // one byte can hold values from 0 to 255

        // checksum calculation
        unsigned char checksum = 0;
        for (int i = 2; i <= 7; i++) checksum += packet[i];
        packet[8] = ~checksum;

        write(fd_, packet, 9);
    }

    float read_current (int servo_id) {
        unsigned char request[8];
        request[0] = 0xFF; // Header 1
        request[1] = 0xFF; // Header 2
        request[2] = (unsigned char)servo_id;
        request[3] = 0x04; // Length: Instruction (1) + Parameters (2) + Checksum (1)
        request[4] = 0x02; // READ Instruction
        request[5] = 0x45; // Current Position Register
        request[6] = 0x02; // Number of bytes to read

        unsigned char checksum = 0;
        for (int i = 2; i <= 6; i++) checksum += request[i];
        request[7] = ~checksum;

        write(fd_, request, 8);

        unsigned char response[8];
        int n = read(fd_, response, sizeof(response));

        if (n < 8) {
            RCLCPP_WARN(this->get_logger(), "Timeout or incomplete packet from servo %d", servo_id);
            return -1.0;

            unsigned char res_checksum = 0;
            
            for (int i = 2; i <= 6; i++) res_checksum += response[i];
            
            if ((unsigned char)~res_checksum != response[7]) {
                RCLCPP_ERROR(this->get_logger(), "Feedback checksum error!");
                return -1.0;
            }
        }

        int raw_current = response[5] | (response[6] << 8);
        if (raw_current > 32767) raw_current -= 65536;
        RCLCPP_INFO(this->get_logger(), "Current read from servo %d: %d", servo_id, raw_current);
        
        return raw_current * 6.5f; // * 6.5 mA
    }

    int translate_hip_logic(float hip_deg) {
        return static_cast<int>((hip_deg / 360.0) * 4096.0 + 1024.0);
    }

    int translate_knee_logic(float knee_deg) {
        return static_cast<int>(1024.0 + ((knee_deg / 360.0) * 4096.0));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrupedTranslatorNode>());
    rclcpp::shutdown();
    return 0;
}