# APIs used for communicating between components

The following APIs are used:
- API for communication between PC application and Lego Technic Hub 1 (lower arm) or Lego Technic Hub 2 (shoulder)
- API for communication between PC application and Arduino (gripper)
- API for communication between PC application and VR app

## API for communication between PC application and Lego Technic Hub 1 or 2:

Trying to keep payloads at max 27 bytes in communication with Lego Hubs, because of Bluetooth packet limitations. Because of this, a proprietary binary communication protocol is used. Angles are in degrees.

Request message from PC to Lego Technic Hub 1 or 2 (the same data is sent to both):
1. Prefix "T"
2. Prefix "W"
3. API version, 1
4. int8, requested movement mode:  
    - 1 = move all joints as fast as possible  
    - 2 = synchronize joints to arrive at the destination simultaneously
    - 3 = calibration mode
5.  int8, desired angle for wrist  
6.  int8, desired angle for underarm 
7.  int8, desired angle for elbow   
8.  int8, desired angle for overarm  
9.  int8, desired angle for shoulder_forward 
10. int8, desired angle for shoulder_out  
11. int8, last known angle for wrist  
12. int8, last known angle for underarm 
13. int8, last known angle for elbow   
14. int8, last known angle for overarm  
15. int8, last known angle for shoulder_forward 
16. int8, last known angle for shoulder_out  
  
Reply message from Lego Technic Hub 1 or 2 to PC:
1. Prefix "T"
2. Prefix "W"
3. API version, 1
4. int8, information source:  
    - 10 = Lego Technic Hub 1 (lower arm), ignore shoulder joint values
    - 11 = Lego Technic Hub 2 (shoulder), ignore lower arm joint values
5.  int8, current angle for wrist (or 0xFF if unknown)
6.  int8, current angle for underarm (or 0xFF if unknown)
7.  int8, current angle for elbow (or 0xFF if unknown)
8.  int8, current angle for overarm (or 0xFF if unknown)
9.  int8, current angle for shoulder_forward (or 0xFF if unknown)
10. int8, current angle for shoulder_out (or 0xFF if unknown)
