#ifndef SRT_CANROUTER_H
#define SRT_CANROUTER_H

#include <Arduino.h>
#include "SRT_Odrive.h"
#include "SRT_CanOpen.h"

enum class Proto { ODRIVE, OPENCAN, UNKNOWN };

struct ParsedId {
    Proto   proto;       // which protocol / MRT type
    uint8_t node_id;     // 0..31 here
    uint8_t cmd_or_func; // reserved, 0 for now
};

ParsedId parse_can_id(uint16_t can_id);

void process_can_frame(uint16_t id, uint8_t len, uint8_t* data,
                       SRT_OdriveMtr& odrive,
                       SRT_CanOpenMtr& opencan);

#endif
