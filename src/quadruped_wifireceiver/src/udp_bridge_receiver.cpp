#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "quadruped_wifireceiver/udp_packet_format.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#ifndef UDP_PORT_RECV
#define UDP_PORT_RECV 8890
#endif

class UDPReceiverNode : public rclcpp::Node {
public:
    UDPReceiverNode() : Node("udp_receiver_node"), last_seq_num_(0) {
        RCLCPP_INFO(this->get_logger(), "Raspberry PI UDP receiver starting...");

        if(!setup_udp_socket()){
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket.");
            return;
        }

        // movement command publisher on raspberry pi
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("Raspberry_cmd_vel", 10);

        // timer for udp packets
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&UDPReceiverNode::receive_timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for UDP packets on port %d...", UDP_PORT_RECV);
    }

    // destructor
    ~UDPReceiverNode() {
        if (sockfd_ != -1) close(sockfd_);
    }

private:
    int sockfd_ = -1;
    uint32_t last_seq_num_;
    struct sockaddr_in host_addr_;
    struct sockaddr_in sender_addr_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool setup_udp_socket(){
        if((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
            perror("Socket creation failed");
            return false;
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000; // 1ms timeout
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        memset(&host_addr_, 0, sizeof(host_addr_));
        host_addr_.sin_family = AF_INET;
        host_addr_.sin_addr.s_addr = INADDR_ANY;
        host_addr_.sin_port = htons(UDP_PORT_RECV);

        if(bind(sockfd_, (struct sockaddr *)&host_addr_, sizeof(host_addr_)) < 0){
            perror("Bind failed");
            return false;
        }
        return true;
    }


    // run this function every 10ms
    void receive_timer_callback(){
        TeleopCommand packet;
        socklen_t addr_len = sizeof(sender_addr_);

        // receive the data
        ssize_t n = recvfrom(sockfd_, &packet, sizeof(packet), 0, 
                             (struct sockaddr *)&sender_addr_, &addr_len);

        if (n > 0) {
            // check sequence number (seq_num)
            if (packet.seq_num <= last_seq_num_ && packet.seq_num != 1) {
                RCLCPP_WARN(this->get_logger(), "Old packet received. Skipping.");
                return;
            }
            last_seq_num_ = packet.seq_num;

            auto vel_msg = geometry_msgs::msg::Twist();
            
            vel_msg.linear.x = packet.linear;
            vel_msg.angular.z = packet.angular;

            vel_publisher_->publish(vel_msg);

            if(packet.seq_num % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "Received packet #%u: lin=%.2f, ang=%.2f", 
                            packet.seq_num, packet.linear, packet.angular);
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPReceiverNode>());
    rclcpp::shutdown();
    return 0;
}