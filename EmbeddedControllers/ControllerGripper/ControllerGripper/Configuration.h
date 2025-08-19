#include <stdint.h>

// Please make a DHCP reservation on your router for this device and the PC, so that they 
// always get the same IP address that can be used to control the gripper.
const uint8_t PC_IP_ADDRESS[] = { 192, 168, 0, 43 }; // IP address of the PC that controls the gripper
const uint16_t ARDUINO_UDP_PORT = 7507; // Port for UDP communication between Arduino and PC (both sending and receiving)

const uint8_t GRIPPER_ANGLE_MIN_DEG = 0;
const uint8_t GRIPPER_ANGLE_MAX_DEG = 90;