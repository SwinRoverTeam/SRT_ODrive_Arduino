### ODrive Arduino CAN Replacement

This is a project to rewrite the CAN aspect of the ODrive Arduino library as I dislike the implementation.

The ODrive documentation has a .dbc file for the messages sent on the can bus, this is in the /assets folder. A DBC viwer such as <a href=https://www.csselectronics.com/pages/dbc-editor-can-bus-database>this</a> will make the file readable.

The current implementation uses function pointers for sending and recieving can messages so that this library can be completely can library independant.