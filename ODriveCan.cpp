#include <ODriveCan.h>


ODriveCanMtr::ODriveCanMtr(MCP_CAN &can_obj, uint8_t node_id) 
{
    _mcp_obj = can_obj;
    _node_id = node_id;

    if (_mcp_obj == NULL) {
        can_attached = false;
    } else {
        can_attached = true;
    }
}

void ODriveCanMtr::begin()
{
    if (_node_id == 0x3f) {
        Serial.println("[ODriveCan - Warning] 0x3f is the broadcast node ID!");
    }
}


/*
    Use when the main code is reading the can messages and passing them to the correct motor
*/
bool ODriveCanMtr::process_cmd(uint8_t cmd, uint8_t len, uint8_t* data)
{

}

/*
    Use when only the motor is on the bus - the library will deal with the MCP
*/
bool ODriveCanMtr::process_msg()
{

}