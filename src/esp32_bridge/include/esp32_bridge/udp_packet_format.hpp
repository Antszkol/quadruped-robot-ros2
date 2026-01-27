#ifndef UDP_PACKET_FORMAT_HPP
#define UDP_PACKET_FORMAT_HPP

#include <cstdint>

#define NUM_JOINTS 8 

// Struktura do sterowania (bez zmian)
struct JointCommandPacket {
    uint32_t seq_num;
    float joint_positions[NUM_JOINTS]; 
    uint8_t status_flag;               
} __attribute__((packed));

// --- ZMIANA: TYLKO GYRO X ---
// Struktura musi mieć dokładnie taki układ jak to, co wysyła ESP32
struct IMUDataPacket {
    uint32_t seq_num;       // 4 bajty
    float gyro_x;           // 4 bajty
} __attribute__((packed));  // Łącznie: 8 bajtów

#endif