#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "quadruped_wifisender/udp_packet_format.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;

// raspberry pi ip address
#define RASPBERRY_IP "192.168.39.83" 

#ifndef UDP_PORT_RECV
#define UDP_PORT_RECV 8890 
#endif

class UDP_Sender_Node : public rclcpp::Node {
public:
    // initialize wifi sender node
    UDP_Sender_Node() : Node("UDP_Sender_Node"), seq_num_counter_(0) {
        RCLCPP_INFO(this->get_logger(), "UDP Sender Node Initialization.");

        if(!setup_udp_socket()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Cannot initiate UDP socket.");
            rclcpp::shutdown();
            return;
        }

        // cmd vel subscription
        teleop_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            1, // queue = 1, keep only the last order
            std::bind(&UDP_Sender_Node::teleop_callback, this, std::placeholders::_1));

        // send orders every 250ms - it's enough
        timer_send_ = this->create_wall_timer(
            250ms, std::bind(&UDP_Sender_Node::send_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Sender ready. Sending UDP every 250ms to %s", RASPBERRY_IP);
    }

    ~UDP_Sender_Node() {
        if (sockfd_ != -1) close(sockfd_);
    }

private:
    int sockfd_ = -1;
    uint32_t seq_num_counter_;
    struct sockaddr_in raspberry_address_;
    
    float last_linear_x_ = 0.0;
    float last_angular_z_ = 0.0;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_send_;

    bool setup_udp_socket() {
        if((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
            perror("UDP socket creation failed!");            
            return false;
        }

        memset(&raspberry_address_, 0, sizeof(raspberry_address_));
        raspberry_address_.sin_family = AF_INET;
        raspberry_address_.sin_port = htons(UDP_PORT_RECV);

        if(inet_aton(RASPBERRY_IP, &raspberry_address_.sin_addr) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Incorrect IP Address: %s", RASPBERRY_IP);
            return false;
        }
        return true;
    }

    // update data from cmd_vel
    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_linear_x_ = msg->linear.x;
        last_angular_z_ = msg->angular.z;
    }

    void send_timer_callback() {
        // sending the packet
        TeleopCommand packet;
        packet.seq_num = ++seq_num_counter_;
        packet.linear = last_linear_x_;
        packet.angular = last_angular_z_;

        ssize_t sendbytes = sendto(
            sockfd_,
            &packet,
            sizeof(packet),
            0,
            (const struct sockaddr*)&raspberry_address_,
            sizeof(raspberry_address_));

        if(sendbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP SENDING ERROR!");
        } else if (packet.seq_num % 4 == 0) { // print every 4 commands
            RCLCPP_INFO(this->get_logger(), "Sent #%u | lin: %.2f, ang: %.2f", 
                        packet.seq_num, packet.linear, packet.angular);
        }
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDP_Sender_Node>());
    rclcpp::shutdown();
    return 0;
}