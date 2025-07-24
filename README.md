# ODrive Arduino CAN Replacement

This is a project to rewrite the CAN aspect of the ODrive Arduino library as I dislike the implementation.

The ODrive documentation has a .dbc file for the messages sent on the can bus, this is in the /assets folder. A DBC viwer such as <a href=https://www.csselectronics.com/pages/dbc-editor-can-bus-database>this</a> will make the file readable. That dbc file and more impementation details can be found [here](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html).

The current implementation uses function pointers for sending can messages so that this library can be completely can library independant.

### Requirements

- This library uses some of the built-in Arduino function, namely millis() and uses Serial for feedback, this can likely be changed to suit another microcontroller if necessary.
- The upper level code will need to handle reading can to send into the motor object(s) and for creating a function that can be passed into the object to allow messages to be sent.

## Implementation

### Sending CAN messages & Constuctor

To implement this library you must create a function that uses the following prototype:

```C++
int send_can (uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);
```
although _send_can_ can be changed to a name of your choice.

can_id is the id of the message that will be sent.\
len is the length of the data array in bytes.\
data is a pointer to an array of bytes that contains the message, it is the same length an len.\
rtr is for if the RTR bit should be set to 0 (false) or 1 (true), see the [RTR](/SRT_ODrive_Arduino/main/README#remote-transmission-request-rtr) section for more details.

The function should return one of four results;
- success (0) - Message was sucessfully sent
- can_bus_fault (1) - There was an issue sending the message
- can_bus_no_response (2) - No nodes acknowledged the message, this can either indicate that the message was not actually put onto the bus or that it was successfuly put onto the bus but there are no nodes that responded
- can_transmission_fail (3) - A more general transmission failure for libraries that do not distingish between a bus fault and no response, use can_bus_fault and can_bus_no_response primarily.

A pointer this function and the node ID of the motor must be passed into the constuctor of the object, for example with a motor with an ID of 0,
```C++
ODriveCanMtr mtr1(&send_can, 0);
```
Details about node ID can be found [here](/SRT_ODrive_Arduino/main/README#node-id).

### Receiving CAN messages

The library **does not** read the CAN bus, so your implementation must read the CAN messages regularly and pass information to the object(s) using the process_msg or process_cmd methods.

process_msg takes the CAN ID directly with no additional effort, if the message is for a different node ID, it will return not_this_node (-1), if the message is for that node, process_cmd will be internally called.

process_cmd requires the node ID to be removed from the CAN ID first, this method relies on you checking which node the message belongs to beforehand, the node ID can be obtained using the node_id method.

### Setting up the motor

Before the motor can be run, it must be configured with the desired control mode, input mode and axis state.

A detailed list of the input modes can be found [here](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode), the control modes can be found [here](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode) and the axis states can be found [here](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState).

Once you know what you want to set the motor to, call the set_cont_mode method passing in the control mode and input mode, and then use the set_axis_state method with the desired axis state.

If you're unsure, use one of the two following presets.

#### Position Control
```C++
mtr1.set_cont_mode(position_control, vel_ramp);
mtr1.set_axis_state(closed_loop_control);
```

use the set_ip_pos method to change position for this control method
#### Velocity Control
```C++
mtr1.set_cont_mode(velocity_control, vel_ramp);
mtr1.set_axis_state(closed_loop_control);
```

Use the set_ip_vel method to change velocity (please note this is revolutions/second as measured at the encoder)

## Background Knowledge

### Controller Area Network (CAN) bus

CAN is a communication standard often used for automotive applications that use a twister pair to send a message. For purposes of this library, only CAN 2.0 needs to be understood, CAN Flexible Data-rate (FD) and CAN XL cannot be used to communicate with ODrive devices.\
Each CAN message has an ID; For standard CAN (2.0a) this is 11 bits (0-2047) and for extended CAN (2.0b) this is 29 bits (0-536,870,911). However, the ODrive standard is built around standard CAN, so extended CAN cannot be used to communicate with the motor, it is probably ok to have extended CAN messages on the same bus, but I haven't tested that.\
There is no "controller" of a CAN network, instead messages are prioritised by their identifier where the lower identifier wins (i.e. ID 0 takes priority over ID 5). In CAN 2.0, all devices must agree on a bit rate with the following being the most common options,
- 125 kbps
- 250 kbps
- 500 kpbs
- 1000 kbps
  
500 kbps is the most common choice as 1000 kbps can become unreliable at distances over 40m.

#### CAN 2.0 Physical Layer (OSI Layer 1)

In CAN, one of the conductors is labelled CAN High (often abreviated to CAN H, CH or just H) with the other CAN Low (CAN L, CL or just L). While no data is being transmitted, CAN Low is kept at the bus voltage with CAN High being kept at 0 V. CAN High and Low act as a differential pair, so changes in the voltage caused by data transmission will cause the two conductors to change in opposite directions (i.e. CAN high changing from 0 V to bus voltage and CAN Low doing the opposite). Changes in voltage where both conductors change in the same direction (i.e. CAN high changing from 0 V to 0.1 V and CAN Low changing from 5 V to 5.1V) are interpreted as noise and are ignored. To ensure that noise will effect both conductors equally, the conductors **must** be a twisted pair, with standard being >1 twist per inch (1 twist per 25.4 mm) with more twists being better but increasing the length of cable needed.

#### Frame structure of standard CAN (OSI Layer 2)

##### Remote Transmission Request (RTR)

### ODrive CAN Standard

#### How CAN ID is used

##### Node ID

##### Command ID
