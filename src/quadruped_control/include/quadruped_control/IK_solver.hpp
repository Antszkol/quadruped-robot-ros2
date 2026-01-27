#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#include <cmath>
#include <algorithm> // Dla std::min i std::max

class IK_solver {
public:
    // Wymiary robota (długości członów nogi)
    double thigh_length; // L1 (udo)
    double shin_length;  // L2 (łydka)

    // Wyniki obliczeń (kąty w radianach)
    // Te zmienne będą odczytywane przez Control Node po wykonaniu obliczeń
    double angle_hip_rad;    // theta1
    double angle_knee_rad;   // theta2

    // Konstruktor
    IK_solver();

    // Główna funkcja licząca
    // Przyjmuje: cel X, cel Y (względem mocowania nogi)
    // Zwraca: 'true' jeśli się udało, 'false' jeśli punkt jest poza zasięgiem
    bool calculateIK(double target_x, double target_y);
};

#endif // IK_SOLVER_HPP