#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
// Upewnij się, że ten plik jest w folderze include/esp32_bridge/
#include "esp32_bridge/udp_packet_format.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#ifndef UDP_PORT_RECV
#define UDP_PORT_RECV 8889 
#endif

class UDPReceiverNode : public rclcpp::Node {
public:
    UDPReceiverNode()
    : Node("udp_bridge_receiver_node"), last_seq_num_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Start Odbiornika UDP (Tylko Gyro X).");

        if (!setup_udp_socket()) {
            RCLCPP_ERROR(this->get_logger(), "Blad tworzenia gniazda UDP.");
            rclcpp::shutdown();
            return;
        }

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);

        // 50 Hz = 20ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&UDPReceiverNode::receive_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Nasluchiwanie na porcie %d...", UDP_PORT_RECV);
    }

    ~UDPReceiverNode() {
        if (sockfd_ != -1) close(sockfd_);
    }

private:
    int sockfd_ = -1;
    struct sockaddr_in host_addr_;
    struct sockaddr_in sender_addr_;
    uint32_t last_seq_num_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool setup_udp_socket()
    {
        if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("Socket failed");
            return false;
        }

        memset(&host_addr_, 0, sizeof(host_addr_));
        host_addr_.sin_family = AF_INET;
        host_addr_.sin_addr.s_addr = INADDR_ANY;
        host_addr_.sin_port = htons(UDP_PORT_RECV);

        if (bind(sockfd_, (const struct sockaddr *)&host_addr_, sizeof(host_addr_)) < 0) {
            perror("Bind failed");
            return false;
        }
        return true;
    }

    void receive_timer_callback()
    {
        IMUDataPacket packet;
        socklen_t addr_len = sizeof(sender_addr_);

        ssize_t len = recvfrom(
            sockfd_, 
            &packet,                  
            sizeof(IMUDataPacket),    
            MSG_DONTWAIT,             
            (struct sockaddr *)&sender_addr_, &addr_len); 

        if (len > 0) {
            // 1. Sprawdzenie rozmiaru (KLUCZOWE: Musi być 8 bajtów)
            if (len != sizeof(IMUDataPacket)) {
                RCLCPP_WARN(this->get_logger(), 
                    "Zly rozmiar pakietu! Dostalem: %zd bajtow, a struktura ma: %lu bajtow. ZAKTUALIZUJ KOD ESP32!", 
                    len, sizeof(IMUDataPacket));
                return;
            }

            last_seq_num_ = packet.seq_num;

            // 2. Wypelnienie wiadomosci ROS
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";

            // TYLKO GYRO X
            imu_msg.angular_velocity.x = (double)packet.gyro_x;
            
            // Reszta zerowa (opcjonalnie mozna jawnie wpisac 0.0)
            imu_msg.angular_velocity.y = 0.0;
            imu_msg.angular_velocity.z = 0.0;
            imu_msg.linear_acceleration.x = 0.0;
            imu_msg.linear_acceleration.y = 0.0;
            imu_msg.linear_acceleration.z = 0.0;

            // 3. Publikacja
            imu_publisher_->publish(imu_msg);

            // 4. Logowanie (co ok. 1 sekunde)
            if (packet.seq_num % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "ODEBRANO: Seq=%d | GyroX=%.4f", packet.seq_num, packet.gyro_x);
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPReceiverNode>());
    rclcpp::shutdown();
    return 0;
}