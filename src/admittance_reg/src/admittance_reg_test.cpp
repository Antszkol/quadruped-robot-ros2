#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class AdmittanceControlNode : public rclcpp::Node {
public:
    AdmittanceControlNode() : Node("adm_reg_node") {
        fd_ = open("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6053386-if00", O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open the port!");
            return;
        }
        setup_serial();

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("leg_telemetry", 10);

        if (!ping_servo(7)) {
            RCLCPP_FATAL(this->get_logger(), "Hardware check failed. Shutting down node.");
            close(fd_);
            return; 
        }

        calibrate_bias();

        last_time_ = this->now();
        timer_ = this->create_wall_timer(10ms, std::bind(&AdmittanceControlNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Admittance node ready to rumble!");
    }

    ~AdmittanceControlNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    uint8_t servo_id = 7;

    const float L = 0.09f;
    const float METERS_PER_TICK = (2.0f * M_PI / 4096.0f) * L;

    // physical params
    const float KT = 3.788f;
    const float ADMITTANCE_GAIN = 4000.0f;
    float M = 4.0f;
    float D = 6.0f;
    float K = 1.4f;
    float ALPHA = 0.05f;
    float DEADZONE = 0.014f; // ignore noise (current feedback from servo is multiplied by 6,5 mA)
    float I_BIAS = 0.0f;

    // EMA bufor
    std::deque<float> sample_buffer_;
    const size_t MEDIAN_SIZE = 5;

    // state variables
    float current_pos_ = 0.0f;
    float home_pos_ = 0.0f;
    float velocity_ = 0.0f;
    float i_filtered_ = 0.0f;
    int fd_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Time last_time_;

    // UART port setup
    void setup_serial() {
        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B1000000);
        cfsetispeed(&tty, B1000000);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_lflag = 0; tty.c_oflag = 0; tty.c_iflag = 0;
        tcsetattr(fd_, TCSANOW, &tty);
    }

    // Insertion sort for EMA
    std::vector<float> perform_insertion_sort(std::vector<float> input) {
        for (size_t i = 1; i < input.size(); i++) {
            float key = input[i];
            int j = i - 1;
            while (j >= 0 && input[j] > key) {
                input[j + 1] = input[j];
                j--;
            }
            input[j + 1] = key;
        }
        return input;
    }

    float get_median_value() {
        if (sample_buffer_.size() < 3) return i_filtered_;

        std::vector<float> to_sort(sample_buffer_.begin(), sample_buffer_.end());
        std::vector<float> sorted = perform_insertion_sort(to_sort);
        
        return sorted[sorted.size() / 2];
    }

    void update_sample_buffer(float new_val) {
        sample_buffer_.push_back(new_val);
        if (sample_buffer_.size() > MEDIAN_SIZE) {
            sample_buffer_.pop_front();
        }
    }

    // current BIAS callibration 
    // calculate the current with no external forces applied to the leg
    void calibrate_bias() {
        RCLCPP_INFO(this->get_logger(), "Kalibracja BIAS... Nie ruszaj nogi!");
        float sum = 0;
        for(int i=0; i<50; i++) {
            float val = read_current_amps(servo_id);
            sum += val;
            update_sample_buffer(val);
            usleep(10000);
        }
        I_BIAS = sum / 50.0f;
        i_filtered_ = I_BIAS;
        RCLCPP_INFO(this->get_logger(), "Bias ustawiony: %.3f A", I_BIAS);
    }

    void control_loop() {
        auto now = this->now();
        float dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;

        // read and filter the current
        float i_raw = read_current_amps(servo_id);
        float i_centered = i_raw - I_BIAS;
        
        if (std::abs(i_centered) < DEADZONE) i_centered = 0.0f;
        update_sample_buffer(i_centered);

        float i_med = get_median_value();
        i_filtered_ = (ALPHA * i_med) + (1.0f - ALPHA) * i_filtered_;
        
        // admittance equation
        float torque_nm = i_filtered_ * KT;
        float virtual_force = torque_nm / L;
        float displacement = current_pos_ - home_pos_;

        float acceleration = (virtual_force - (D * velocity_) - (K * displacement)) / M;
        velocity_ += acceleration * dt;
        current_pos_ += velocity_ * dt;

        float servo_cmd = 3072.0f + (current_pos_ / METERS_PER_TICK);

        // clamping for the servo, send the command
        servo_cmd = std::clamp(servo_cmd, 2047.0f, 4095.0f);
        send_goal_pos(servo_id, (uint16_t)std::round(servo_cmd));

        // publish data
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = { (double)i_filtered_, (double)i_med, (double)i_raw, (double)acceleration};
        publisher_->publish(msg);
    }

    // ping servo
    bool ping_servo(uint8_t id) {
        tcflush(fd_, TCIFLUSH);

        uint8_t packet[6] = {0xFF, 0xFF, id, 0x02, 0x01, 0};
        
        uint8_t cs = 0;
        for (int i = 2; i < 5; i++) cs += packet[i];
        packet[5] = ~cs;

        if (write(fd_, packet, 6) < 0) return false;

        usleep(2000);

        uint8_t response[6];
        int n = read(fd_, response, 6);

        if (n >= 6 && response[0] == 0xFF && response[1] == 0xFF && response[2] == id) {
            RCLCPP_INFO(this->get_logger(), "Servo ID %d: CONNECTED", id);
            return true;
        }

        RCLCPP_ERROR(this->get_logger(), "Servo ID %d: NOT RESPONDING!", id);
        return false;
    }

    // send position command to the servo
    void send_goal_pos(uint8_t id, uint16_t pos) {
        uint8_t p[9] = {0xFF, 0xFF, id, 0x05, 0x03, 0x2A, (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8), 0};
        uint8_t cs = 0;
        for (int i = 2; i < 8; i++) cs += p[i];
        p[8] = ~cs;
        write(fd_, p, 9);
    }

    // read current [A]
    float read_current_amps(uint8_t id) {
        tcflush(fd_, TCIFLUSH);
        uint8_t req_i[] = {0xFF, 0xFF, id, 0x04, 0x02, 0x45, 0x02, 0};
        uint8_t cs_i = 0; for(int j=2; j<7; j++) cs_i += req_i[j]; req_i[7] = ~cs_i;
        write(fd_, req_i, 8);
        usleep(4000); 

        float current_val = 0.0f;
        uint8_t res_i[8];
        if (read(fd_, res_i, 8) >= 8) {
            int16_t raw_i = (res_i[5] | (res_i[6] << 8)) & 0x03FF;
            current_val = (raw_i * 6.5f) / 1000.0f;
        } else { return -1.0f; }

        uint8_t req_l[] = {0xFF, 0xFF, id, 0x04, 0x02, 0x3C, 0x02, 0};
        uint8_t cs_l = 0; for(int j=2; j<7; j++) cs_l += req_l[j]; req_l[7] = ~cs_l;
        write(fd_, req_l, 8);
        usleep(4000);

        uint8_t res_l[8];
        if (read(fd_, res_l, 8) >= 8) {
            int16_t raw_l = res_l[5] | (res_l[6] << 8);
            if (raw_l & 0x0400) current_val *= -1.0f;
        }
        return current_val;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdmittanceControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}