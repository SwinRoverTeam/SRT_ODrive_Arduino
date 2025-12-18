#include "SRT_OpenCan.h"

SRT_CanOpenMtr::SRT_CanOpenMtr(
    int (*send_func)(uint16_t, uint8_t, uint8_t*, bool),
    uint8_t node_id
) {
    _node_id = node_id;
    can_send_msg = send_func;
}

int SRT_CanOpenMtr::process_msg(uint16_t can_id, uint8_t len, uint8_t* data) {
    if ((can_id & 0x7F) != _node_id) return -1;  // Not our node
    // TODO: parse SDO/PDO responses later
    return 0;
}

int SRT_CanOpenMtr::send_sdo_write(uint16_t index, uint8_t sub, uint32_t value, uint8_t size) {
    uint16_t can_id = 0x600 + _node_id;
    uint8_t cs = (size == 1) ? 0x2F : (size == 2) ? 0x2B : 0x23;

    uint8_t data[8] = {
        cs,
        static_cast<uint8_t>(index & 0xFF),
        static_cast<uint8_t>((index >> 8) & 0xFF),
        sub,
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF)
    };

    return can_send_msg(can_id, 8, data, false);
}

int SRT_CanOpenMtr::enable_motor() {
    send_sdo_write(0x6040, 0x00, 6, 2);
    delay(10);
    send_sdo_write(0x6040, 0x00, 7, 2);
    delay(10);
    return send_sdo_write(0x6040, 0x00, 15, 2);
}

int SRT_CanOpenMtr::move_relative(int32_t steps, uint32_t accel_ms, uint32_t decel_ms) {

    send_sdo_write(0x6060, 0x00, 1, 1);  // Position mode
    send_sdo_write(0x6081, 0x00, 1000, 4);// Profile velocity 1000
    send_sdo_write(0x6083, 0x00, accel_ms, 4);
    send_sdo_write(0x6084, 0x00, decel_ms, 4);
    send_sdo_write(0x6040, 0x00, 15, 2);  // Ensure enabled
    send_sdo_write(0x607A, 0x00, (uint32_t)steps, 4);  // Target position
    return send_sdo_write(0x6040, 0x00, 95, 2);  // Start relative move
}

// ============= Stuff I added for serial ============== all below

void SRT_CanOpenMtr::handleSerialCommand(const String &cmd, SRT_CanOpenMtr* motors, size_t num_Openmotors, const uint8_t* node_ids, uint32_t accel_ms, uint32_t decel_ms) {
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("help")) { // when help typed it diaplayes help
        Serial.println("Commands:");
        Serial.println("  list");
        Serial.println("  m<idx> <pos>");
        Serial.print  ("Fixed acc/dec: ");
        Serial.print  (accel_ms);
        Serial.print  (", ");
        Serial.println(decel_ms);
        Serial.println("Examples: m0 10000   or   m1 -20000");
        return;
    }

    if (cmd.equalsIgnoreCase("list")) { // when list typed shows list of CAN IDs
        Serial.println("LiChuan nodes:");
        for (size_t i = 0; i < num_Openmotors; ++i) {
            Serial.print("  index ");
            Serial.print(i);
            Serial.print(" -> CAN ID ");
            Serial.println(node_ids[i]);
        }
        return;
    } 
    

    // Expect commands like "m0 10000"
    if (cmd.charAt(0) == 'm' || cmd.charAt(0) == 'M') {
        int space1 = cmd.indexOf(' ');
        if (space1 < 0) {
            Serial.println("ERR: usage m<idx> <pos>");
            return;
        }

        String motorStr = cmd.substring(1, space1); //reading from second digit and beyond
        int idx = motorStr.toInt(); //the index of the motor the m1 (the 1)
        if (idx < 0 || idx >= (int)num_Openmotors) { 
            Serial.println("ERR: motor index out of range");
            return;
        }

        String posStr = cmd.substring(space1 + 1); //reading from beyond the first 2 digits
        posStr.trim();
        if (posStr.length() == 0) {
            Serial.println("ERR: missing position");
            return;
        }

        long targetPos = posStr.toInt(); //converts to integer from string

        // Update only target position; acc/dec are fixed
        motors[idx].move_relative((int32_t)targetPos, accel_ms, decel_ms);

        Serial.print("OK: motor index "); //prints what has been sent in serial for our human brains to understand
        Serial.print(idx);
        Serial.print(" (CAN ID ");
        Serial.print(node_ids[idx]);
        Serial.print(") -> target ");
        Serial.print(targetPos);
        Serial.print(" (acc ");
        Serial.print(accel_ms);
        Serial.print(", dec ");
        Serial.print(decel_ms);
        Serial.println(")");
        return;
    }

    Serial.println("Unknown command. Type 'help'.");
}


String SRT_CanOpenMtr::readSerialLine() {
    String input = "";
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') break;
        input += c;
    }
    input.trim();
    return input;
}
