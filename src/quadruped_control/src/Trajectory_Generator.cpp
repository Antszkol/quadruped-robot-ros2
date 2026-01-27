#include "quadruped_control/Trajectory_Generator.hpp" 
#include "quadruped_control/quadruped_config.hpp" 
#include "rclcpp/rclcpp.hpp"
#include <cmath> 

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

TrajectoryGenerator::TrajectoryGenerator(double step_len, double ground_h, double duration, double height) {
    this->step_length_x = step_len;
    this->ground_y = ground_h;
    this->step_duration = duration;
    this->step_height = height;
    this->time_elapsed = 0.0;
    
    // step initialization
    this->step_start_x = step_len / 2.0;
    this->step_end_x = -step_len / 2.0;
    this->is_swing_phase = false; 
}

bool TrajectoryGenerator::calculateNextPoint(double dt, double &out_x, double &out_y) {
    this->time_elapsed += dt;

    // progress ration for height calculation
    double progress_ratio = this->time_elapsed / this->step_duration;

    // if the cycle has finished, 
    // set the last points in the trajectory
    if (progress_ratio >= 1.0) {
        out_x = this->step_end_x;
        out_y = this->ground_y;
        this->time_elapsed = 0.0;
        return false;
    } 
    // if the cycle is ongoing, calculate next point
    else {
        double smooth_x_ratio = progress_ratio - (std::sin(2.0 * M_PI * progress_ratio) / (2.0 * M_PI));
        double step_span = this->step_end_x - this->step_start_x;
        out_x = this->step_start_x + (smooth_x_ratio * step_span);
        
        if(this->is_swing_phase) {
            double arc_height = this->step_height * std::sin(M_PI * progress_ratio);
            out_y = this->ground_y + arc_height;
        }
        else {
            out_y = this->ground_y;
        }

        return true;
    }
}

void TrajectoryGenerator::start_next_cycle(double current_x, double calculated_step_len, double ground_h) {
    this->toggle_phase();
    double next_target_x = 0.0;

    // set new targets
    if (this->is_swing_phase) {
        next_target_x = -calculated_step_len / 2.0; 
    } else {
        next_target_x = calculated_step_len / 2.0;
    }
    this->set_target(current_x, next_target_x, ground_h);
}

void TrajectoryGenerator::set_target(double start_x, double target_x, double target_y) {
    this->step_start_x = start_x;
    this->step_end_x   = target_x;
    this->ground_y     = target_y;
    this->time_elapsed = 0.0;
}
void TrajectoryGenerator::toggle_phase() { this->is_swing_phase = !this->is_swing_phase; }
bool TrajectoryGenerator::get_phase() const { return this->is_swing_phase; }