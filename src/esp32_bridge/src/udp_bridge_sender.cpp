#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "esp32_bridge/udp_packet_format.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define ESP32_IP "192.168.16.68"

#ifndef UDP_PORT_RECV
#define UDP_PORT_RECV 8890 
#endif

class UDPSenderNode : public rclcpp::Node {
public:
    UDPSenderNode()
    : Node("udp_bridge_sender_node"), seq_counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "UDP bridge initialization (Sender).");
        
        // 1. Inicjalizacja Gniazda UDP
        if (!setup_udp_socket()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Can not initiate an UDP socket.");
            rclcpp::shutdown();
            return;
        }

        // 2. Subskrypcja poleceń kątów
        joint_command_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/angles_modified", 10,
            std::bind(&UDPSenderNode::joint_command_callback, this, std::placeholders::_1)); 

        RCLCPP_INFO(this->get_logger(), "Waiting for orders on topic /angles_modified...");
    }

    ~UDPSenderNode() {
        if (sockfd_ != -1) {
            close(sockfd_);
        }
    }

private:
    int sockfd_ = -1;
    struct sockaddr_in esp32_addr_;
    
    // --- CO DODAŁEM (2): Licznik sekwencji ---
    uint32_t seq_counter_; 

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_;

    bool setup_udp_socket()
    {
        // a. Utworzenie gniazda
        if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
            perror("UDP socket creation failed!");            
            return false;
        }

        // b. Konfiguracja adresu docelowego (ESP32)
        memset(&esp32_addr_, 0, sizeof(esp32_addr_)); 
        esp32_addr_.sin_family = AF_INET;             
        
        // --- CO ZMIENIŁEM (3): Usunąłem błędną linię z host_addr_ ---
        // Sender nie musi bindować się do INADDR_ANY, on tylko wysyła na konkretny adres.
        
        // Ustawiamy Port Docelowy (CMD)
        esp32_addr_.sin_port = htons(UDP_PORT_RECV);  
        
        // Konwersja String IP -> Binary IP
        if (inet_aton(ESP32_IP, &esp32_addr_.sin_addr) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Incorrect IP address: %s", ESP32_IP); 
            return false;                                                           
        }
        
        RCLCPP_INFO(this->get_logger(), "UDP socket created, destination: %s:%d", ESP32_IP, UDP_PORT_RECV);
        return true;
    }

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr angles_modified)
    {
        // Zabezpieczenie przed pustą wiadomością lub zbyt małą liczbą stawów
        if (angles_modified->position.size() < NUM_JOINTS) {
            RCLCPP_WARN(this->get_logger(), "Za malo stawow! Otrzymano: %zu, Oczekiwano: %d", 
                angles_modified->position.size(), NUM_JOINTS);
            return;
        }

        // 1. Wypełnienie struktury pakietu
        JointCommandPacket packet;
        
        // --- CO ZMIENIŁEM (4): Poprawna nazwa pola w strukturze ---
        packet.seq_num = seq_counter_++; // W .hpp struktura ma pole 'seq_num', nie 'last_seq_num_'
        packet.status_flag = 1; // 1 = Ruch (przykładowo)

        for (int i = 0; i < NUM_JOINTS; ++i) {
            packet.joint_positions[i] = (float)angles_modified->position[i]; 
        }
        
        // 2. Wysłanie pakietu
        ssize_t sent_bytes = sendto(
            sockfd_, 
            &packet, 
            sizeof(JointCommandPacket), 
            0, 
            (const struct sockaddr *)&esp32_addr_, 
            sizeof(esp32_addr_)); 

        if (sent_bytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "BLAD WYSYLANIA UDP!");
        } else {
            // Debug co 50 pakietów
            if (packet.seq_num % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "Sent CMD #%u -> J1: %.2f", packet.seq_num, packet.joint_positions[0]);
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPSenderNode>());
    rclcpp::shutdown();
    return 0;
}