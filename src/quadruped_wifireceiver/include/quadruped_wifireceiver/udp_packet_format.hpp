#ifndef UDP_PACKET_FORMAT_HPP
#define UDP_PACKET_FORMAT_HPP

#include <cstdint>

#define NUM_JOINTS 8 

// Struktura do sterowania (bez zmian)
struct TeleopCommand {
    uint32_t seq_num = 0;
    float linear;
    float angular;           
} __attribute__((packed));

#endif