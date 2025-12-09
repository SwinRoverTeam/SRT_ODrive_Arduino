#ifndef SRT_OpenCan_H
#define SRT_OpenCan_H

#include <Arduino.h>
#include "driver/twai.h"

class SRT_CanOpenMtr {
private:
    uint8_t _node_id;  // 0..31 (your “local” node id)
    int (*can_send_msg)(uint16_t, uint8_t, uint8_t*, bool);

    int send_sdo_write(uint16_t index, uint8_t sub, uint32_t value, uint8_t size);

public:
    SRT_CanOpenMtr(int (*send_func)(uint16_t, uint8_t, uint8_t*, bool),
                   uint8_t node_id);

    // Called from router when a CAN frame is for CANopen
    int process_msg(uint16_t can_id, uint8_t len, uint8_t* data);

    // Simple profile position / velocity helpers
    int enable_motor();                               // 6 -> 7 -> 15 at 0x6040
    int set_profile_position(int32_t pos, uint32_t vel_rpm, uint32_t accel_ms, uint32_t decel_ms);
    int set_profile_velocity(uint32_t vel_rpm, uint32_t accel_ms, uint32_t decel_ms);
};

#endif