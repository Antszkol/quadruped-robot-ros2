#include "quadruped_control/IK_solver.hpp" 
#include "quadruped_control/quadruped_config.hpp" 
#include <iostream>
#include <cmath>
#include <algorithm>

const double RAD_TO_DEG = 57.2958;

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// set legs' length
IK_solver::IK_solver() {
    this->thigh_length = RobotConfig::THIGH_LENGTH; 
    this->shin_length  = RobotConfig::SHIN_LENGTH;  
    
    this->angle_hip_rad = 0.0;
    this->angle_knee_rad = 0.0;
}

// IK calculation
bool IK_solver::calculateIK(double target_x, double target_y) {
    // hip <-> feet vector length
    double D_sq = target_x * target_x + target_y * target_y;
    double D = std::sqrt(D_sq); 

    // check if it's within the reachable distance
    if (D > (this->thigh_length + this->shin_length) || 
        D < std::abs(this->thigh_length - this->shin_length)) {
        return false;
    }

    // beta angle = knee angle
    double cos_beta = (this->thigh_length * this->thigh_length + 
                       this->shin_length * this->shin_length - D_sq) / 
                      (2 * this->thigh_length * this->shin_length);
    cos_beta = std::clamp(cos_beta, -1.0, 1.0);
    double beta = std::acos(cos_beta) * RAD_TO_DEG;
    
    // for the knee servo, 180 degrees mean vertical position 
    this->angle_knee_rad = 180.0 - beta; 

    // alpha + gamma angles = hip angle
    double cos_alpha = (D_sq + this->thigh_length * this->thigh_length - 
                        this->shin_length * this->shin_length) / 
                       (2 * D * this->thigh_length);
    cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
    double alpha = std::acos(cos_alpha) * RAD_TO_DEG;
    double gamma = std::atan2(target_x, -target_y) * RAD_TO_DEG;

    // final hip angle
    this->angle_hip_rad = 90.0 - (gamma + alpha); 

    return true;
}