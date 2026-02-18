**Quadruped Robot - ROS 2 Control System**

This repository contains the complete control system for a 8-DOF quadruped robot, developed as an engineering thesis at the Warsaw University of Technology. The project integrates low-level hardware communication with high-level trajectory planning using ROS 2.
Hardware Specifications

    Master Controller: Raspberry Pi 4 4GB RAM.

    Actuators: 8x Feetech STS3215 Serial Bus Servos (UART communication).

    Power Supply: 11.1V 5000mAh Li-Ion battery with XL4015 step-down converter.

    Mechanical Structure: 3D-printed ABS body featuring a parallel linkage mechanism for the legs.

**Core Packages**

    quadruped_control (C++): Core logic implementing the Inverse Kinematics (IK) solver and the gait trajectory generator (swing and stance phases).

    quadruped_servotranslator_cpp (C++): High-performance bridge that translates joint angles into servo-specific binary values and monitors real-time feedback (current, position).

    quadruped_wifireceiver (C++): UDP-based communication bridge for low-latency remote teleoperation.

**Getting Started**

Since the project is in the prototype phase, nodes are currently launched manually via the terminal.
**1. UDP Receiver (Robot)**

Start the network bridge to listen for remote commands:
Bash

ros2 run quadruped_wifireceiver udp_receiver_node

**2. Servo Translator (Robot)**

Initialize the hardware interface for the ST3215 servos:
Bash

ros2 run quadruped_servotranslator_cpp servotranslator_node

**3. Control Node (Robot)**

Launch the main gait engine and IK solver:
Bash

ros2 run quadruped_control quadruped_control_node

**4. Teleop (PC)**

Run the keyboard controller on your workstation:
Bash

ros2 run quadruped_teleop teleop_keyboard

**Key Algorithms**

    Inverse Kinematics: Analytical solution for a 2-DOF leg geometry.

    Trajectory Generation: Smooth foot trajectory using a parametric sine-wave mapping (smooth_x_ratio) to minimize mechanical vibration.

    Current Feedback: Real-time monitoring via register 0x45 for load estimation and admittance control planning.

**UPDATES**  

**Admittance regulation**
      
Admittance regulation described with the formula below:  
    
$$F_{ext} = M\ddot{x} + D\dot{x} + Kx$$

Was implemented in the code, aiming to make the robot react to uneven enviroment by adjusting leg trajectories to keep body on a constant height.
