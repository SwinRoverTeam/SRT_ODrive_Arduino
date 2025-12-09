
#include "SRT_OpenCan.h"
#include <Arduino.h>
#include "driver/twai.h"

SRT_CanOpenMtr::SRT_CanOpenMtr(
    int (*send_func)(uint16_t, uint8_t, uint8_t*, bool),
    uint8_t node_id
) : _node_id(node_id), can_send_msg(send_func) {}

int SRT_CanOpenMtr::process_msg(uint16_t can_id, uint8_t len, uint8_t* data) {
    // CANopen base uses 11‑bit ID: [10:7]=function code, [6:0]=nodeID
    uint8_t node = can_id & 0x7F;
    if (node != _node_id && node != 0) return -1;  // 0 is broadcast

    // For now just accept, you can later decode PDO/SDO replies if needed.
    (void)len;
    (void)data;
    return 0;
}

int SRT_CanOpenMtr::send_sdo_write(uint16_t index, uint8_t sub,
                                   uint32_t value, uint8_t size) {
    uint16_t can_id = 0x600 + _node_id;

    uint8_t cs;
    switch (size) {
        case 1: cs = 0x2F; break;
        case 2: cs = 0x2B; break;
        case 4: cs = 0x23; break;
        default:
            Serial.println("[SRT_CanOpen] invalid SDO size");
            return -1;
    }

    uint8_t data[8] = {
        cs,
        (uint8_t)(index & 0xFF),
        (uint8_t)((index >> 8) & 0xFF),
        sub,
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF)
    };

    return can_send_msg(can_id, 8, data, false);
}

int SRT_CanOpenMtr::enable_motor() {
    // 6 -> 7 -> 15 @ 0x6040 as per manual
    send_sdo_write(0x6040, 0x00, 6, 2);
    delay(10);
    send_sdo_write(0x6040, 0x00, 7, 2);
    delay(10);
    return send_sdo_write(0x6040, 0x00, 15, 2);
}

int SRT_CanOpenMtr::set_profile_position(int32_t pos,
                                         uint32_t vel_rpm,
                                         uint32_t accel_ms,
                                         uint32_t decel_ms) {
    // 1) mode = position (6060 = 1)
    send_sdo_write(0x6060, 0x00, 1, 1);

    // 2) motion params (3.2.1)
    send_sdo_write(0x6081, 0x00, vel_rpm, 4);
    send_sdo_write(0x6083, 0x00, accel_ms, 4);
    send_sdo_write(0x6084, 0x00, decel_ms, 4);

    // 3) target position
    send_sdo_write(0x607A, 0x00, (uint32_t)pos, 4);

    // 4) start absolute move: 15 -> 31 (bit4 rising edge)
    return send_sdo_write(0x6040, 0x00, 31, 2);
}

int SRT_CanOpenMtr::set_profile_velocity(uint32_t vel_rpm,
                                         uint32_t accel_ms,
                                         uint32_t decel_ms) {
    // 1) mode = profile speed (6060 = 3)[1]
    send_sdo_write(0x6060, 0x00, 3, 1);

    // 2) parameters
    send_sdo_write(0x60FF, 0x00, vel_rpm, 4); //0x60ff is index for target velocity, 0x00 is subindex,
    send_sdo_write(0x6083, 0x00, accel_ms, 4);
    send_sdo_write(0x6084, 0x00, decel_ms, 4);

    // 3) enable sequence (6,7,15) if not already done
    return enable_motor();
}


