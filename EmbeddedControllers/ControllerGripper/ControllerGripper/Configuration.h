#include <stdint.h>
#include "IPAddress.h"

// Please make a DHCP reservation on your router for this device and the PC, so that they 
// always get the same IP address that can be used to control the gripper.
const IPAddress PC_IP_ADDRESS = IPAddress( 192, 168, 0, 43 ); // IP address of the PC that controls the gripper
const uint16_t ARDUINO_UDP_PORT = 7507; // Port for UDP communication between Arduino and PC (both sending and receiving)

const uint8_t GRIPPER_ANGLE_MIN_DEG = 0;
const uint8_t GRIPPER_ANGLE_MAX_DEG = 45;

const int16_t SEND_PERIOD_MS = 100; // How often to send current gripper angle to the PC app
