#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "quadruped_control/quadruped_config.hpp" 
#include "quadruped_control/IK_solver.hpp"
#include "quadruped_control/Trajectory_Generator.hpp"

using namespace std::chrono_literals;

const int NUM_LEGS = 4;

class QuadrupedControlNode : public rclcpp::Node {
public:
    QuadrupedControlNode() : Node("quadruped_control_node") {
        // teleop subscription 
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&QuadrupedControlNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // gait parameters subscription
        gait_parameters_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "gait_params", 10, std::bind(&QuadrupedControlNode::gait_params_callback, this, std::placeholders::_1));

        // publish the angles
        pub_joints_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_group_command", 10);

        // timer creation
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(RobotConfig::DT), 
            std::bind(&QuadrupedControlNode::control_loop, this));

        for(int i=0; i<NUM_LEGS; i++) {
            // create 4 IK solvers, one for each leg
            ik_solvers_.push_back(std::make_unique<IK_solver>()); 
            
            // create 4 trajectory generators, one for each leg
            trajectory_generators_.push_back(std::make_unique<TrajectoryGenerator>(
                RobotConfig::BASE_STEP_LENGTH, 
                RobotConfig::DEFAULT_GROUND_Y,
                RobotConfig::STEP_DURATION,
                RobotConfig::STEP_HEIGHT
            ));
        }

        RCLCPP_INFO(this->get_logger(), "Quadruped Control START! [%2f] Waiting for orders...", RobotConfig::LOOP_RATE_HZ);
    }

private:
    double target_linear_x_ = 0.0;
    double target_angular_z_ = 0.0;

    double pending_base_step_length_ = RobotConfig::BASE_STEP_LENGTH;
    double pending_step_height_ = RobotConfig::STEP_HEIGHT;
    double pending_turn_factor = RobotConfig::TURN_FACTOR;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gait_parameters_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_joints_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::unique_ptr<IK_solver>> ik_solvers_;
    std::vector<std::unique_ptr<TrajectoryGenerator>> trajectory_generators_; 

    // cmd_vel topic callback function - update targets
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        target_linear_x_ = msg->linear.x;
        target_angular_z_ = msg->angular.z;
        RCLCPP_INFO(this->get_logger(), "ANGULAR VALUE FROM CMD VEL: %.3f", target_angular_z_);
    }

    // gait parameters callback function - update movement parameters
    void gait_params_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        pending_base_step_length_ = msg->data[0]; 
        pending_step_height_ = msg->data[1];
        pending_turn_factor = msg->data[2];
        }
    }

    // main loop
    void control_loop() {
    std_msgs::msg::Float64MultiArray commands;

    // check if there is any command input
    bool is_moving_input = (std::abs(target_linear_x_) > 0.01 || std::abs(target_angular_z_) > 0.01);

    // for each leg:
    for (int i = 0; i < NUM_LEGS; ++i) {
        double current_x, current_y;

        // check if the leg is currently stepping
        bool is_stepping = trajectory_generators_[i]->calculateNextPoint(RobotConfig::DT, current_x, current_y);

        if (is_moving_input) {
            double calc_len = RobotConfig::BASE_STEP_LENGTH * target_linear_x_;
            
            // this loop runs only once at the start of the node
            if (trajectory_generators_[i]->start_flag == false) {
                trajectory_generators_[i]->start_flag = true;

                // toggle phase for diagonal legs
                if (i == 0 || i == 3) trajectory_generators_[i]->toggle_phase();
                
                // if the leg is in SWING phase, target is forward (+), else backward (-)
                double target_x = (trajectory_generators_[i]->get_phase()) ? (calc_len / 2.0) : (-calc_len / 2.0);
                
                RCLCPP_INFO(this->get_logger(), "Leg %d: TargetX=%.3f, Phase=%s", 
                            i, target_x, trajectory_generators_[i]->get_phase() ? "SWING" : "STANCE");

                // set the initial target for the leg
                trajectory_generators_[i]->set_target(current_x, target_x, RobotConfig::DEFAULT_GROUND_Y);
            }
        }

        // if the leg is not currently stepping, start the next cycle or hold position
        if (!is_stepping) {
            if (is_moving_input) {
                double updated_step_length = target_linear_x_ * pending_base_step_length_;

                if(target_linear_x_ == 0.0 & target_angular_z_ != 0.0){
                    updated_step_length = 1.0 * pending_base_step_length_;
                }

                if(target_angular_z_ > 0.0 & target_linear_x_ == 0.0){
                    if(i == 1 || i == 3){ // right legs - mirror the direction
                        updated_step_length = updated_step_length * pending_turn_factor;
                    }
                    else{
                        updated_step_length = updated_step_length * pending_turn_factor * (-1);
                    }
                }
                
                if(target_angular_z_ < 0.0 & target_linear_x_ == 0.0){
                    if(i == 1 || i == 3){ // right legs - mirror the direction
                        updated_step_length = updated_step_length * pending_turn_factor * (-1);
                    }
                    else{
                        updated_step_length = updated_step_length * pending_turn_factor;
                    }
                }

                // start new movement cycle
                trajectory_generators_[i]->start_next_cycle(current_x, updated_step_length, RobotConfig::DEFAULT_GROUND_Y);
            } 
            else {
                trajectory_generators_[i]->start_flag = false;
                if (trajectory_generators_[i]->get_phase()) trajectory_generators_[i]->toggle_phase();
                trajectory_generators_[i]->set_target(current_x, 0.0, RobotConfig::DEFAULT_GROUND_Y);
            }
        }

        double ik_x_input = current_x; 

        // calculate IK for the current target position
        if (ik_solvers_[i]->calculateIK(ik_x_input, current_y)) {
            commands.data.push_back(ik_solvers_[i]->angle_hip_rad);
            commands.data.push_back(ik_solvers_[i]->angle_knee_rad);
        }
        else {
            commands.data.push_back(ik_solvers_[i]->angle_hip_rad);
            commands.data.push_back(ik_solvers_[i]->angle_knee_rad);
        }
    }
    pub_joints_->publish(commands);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrupedControlNode>());
    rclcpp::shutdown();
    return 0;
}