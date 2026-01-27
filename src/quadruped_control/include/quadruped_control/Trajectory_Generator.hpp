#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include <cmath>

class TrajectoryGenerator {
private:
    // Pola prywatne
         // <--- Dodałem to (Error 1)
    double ground_y;
    double step_duration;
    double time_elapsed;
    bool is_swing_phase;

public:
    double step_height;
    double step_length_x;
    double step_end_x;
    double step_start_x;
    bool start_flag = false;
    // Konstruktor
    TrajectoryGenerator(double step_len, double ground_h, double duration, double height);

    // Główna metoda obliczeniowa
    bool calculateNextPoint(double dt, double &out_x, double &out_y);

    // Metody pomocnicze
    void reset_step();
    void set_target(double start_x, double target_x, double ground_h);
    
    // <--- Dodałem deklarację tej nowej funkcji (Error 2)
    void start_next_cycle(double current_x, double calculated_step_len, double ground_h);

    void toggle_phase();
    
    // Getter fazy (z const)
    bool get_phase() const;

private:
    int log_counter = 0;
};

#endif // TRAJECTORY_GENERATOR_HPP