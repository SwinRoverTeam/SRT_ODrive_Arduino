#include "SRT_CanRouter.h"
#include <Arduino.h>
#include "driver/twai.h"
#include "SRT_OpenCan.h"

ParsedId parse_can_id(uint16_t can_id) {
    // proto = UNKNOWN, node_id = 0, cmd_or_func = 0
    ParsedId p = {Proto::UNKNOWN, 0, 0};

    // MRT type = bits 6:5
    uint8_t type    = (can_id >> 5) & 0x3;   // 0..3
    // Node ID = bits 4:0
    uint8_t node_id =  can_id        & 0x1F; // 0..31

    switch (type) {
        case 0b00:  // Lichuan / CANopen
            // Valid node_id range: 0..31
            p.proto   = Proto::OPENCAN;
            p.node_id = node_id;
            break;

        case 0b01:  // GIM / ODrive, nodes 1+4n
            // Valid node_id values: 1,5,9,13,17,21,25,29
            if (node_id >= 1 && ((node_id - 1) % 4 == 0)) {
                p.proto   = Proto::ODRIVE;
                p.node_id = node_id;
            }
            break;

        case 0b10:  // future type 3
        case 0b11:  // future type 4
        default:
            // Leave as UNKNOWN so router ignores these for now
            break;
    }

    p.cmd_or_func = 0;
    return p;
}

void process_can_frame(uint16_t id, uint8_t len, uint8_t* data,
                       SRT_OdriveMtr& odrive,
                       SRT_CanOpenMtr& opencan) {
    ParsedId p = parse_can_id(id);

    switch (p.proto) {
        case Proto::ODRIVE:
            odrive.process_msg(id, len, data);
            break;

        case Proto::OPENCAN:
            opencan.process_msg(id, len, data);
            break;

        case Proto::UNKNOWN:
        default:
            // ignore for now
            break;
    }
}
