#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread> 
#include <chrono>


const double DT = 0.016;

class AdmittanceController {
public:
    double M = 1.0;    // Mass
    double D = 10.0;   // Damper
    double K = 200.0;  // K constant for Spring

    double pos_y = 0.0; // Current x
    double vel_y = 0.0; // Aktualna prędkość (v) - "ZMYŚLONA PRĘDKOŚĆ"

    // Constructor
    AdmittanceController() { reset(); }

    void reset() {
        pos_y = 0.0;
        vel_y = 0.0;
    }

    double compute(double measured_force) {
        
        double F_net = measured_force - (D * vel_y) - (K * pos_y);
        double acceleration = F_net / M;

        vel_y = vel_y + (acceleration * DT);
        pos_y = pos_y + (vel_y * DT);

        pos_y = std::clamp(pos_y, -0.05, 0.05);

        return pos_y;
    }
};

int main() {
    AdmittanceController controller;

    std::cout << "START SYMULACJI (60Hz)\n";
    std::cout << "Czas[s] | Sila[N] | Ugiecie[m] | Predkosc[m/s]\n";
    std::cout << "----------------------------------------------\n";

    double current_time = 0.0;

    for (int i = 0; i < 100; i++) {
        double force = 0.0;
        
        if (current_time > 0.2 && current_time < 0.4) {
            force = 20.0; 
        }

        double ugiecie = controller.compute(force);

        if (i % 5 == 0) {
            printf("%.3f   | %6.1f  | %+7.4f    | %+7.4f\n", 
                   current_time, force, ugiecie, controller.vel_y);
        }

        current_time += DT;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "----------------------------------------------\n";
    std::cout << "KONIEC. Widziales jak ugiecie roslo i wracalo?\n";

    return 0;
}