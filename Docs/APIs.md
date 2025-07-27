# APIs used for communicating between components

The following APIs are used:
- API for communication between PC application and Lego Technic Hub 1 (lower arm) or Lego Technic Hub 2 (shoulder)
- API for communication between PC application and Arduino (gripper)
- API for communication between PC application and VR app

## API for communication between PC application and Lego Technic Hub 1 or 2, and Arduino:

Want to keep payloads short when communication with Lego Hubs, because of Bluetooth packet limitations (source: https://github.com/pybricks/technical-info/blob/master/pybricks-ble-broadcast-observe.md). Because of this, a proprietary binary communication protocol is used. Angles are in degrees.

Request message from PC to Lego Technic Hub 1 or 2 or Arduino (the same data is sent to all):

1. Prefix "T"
2. Prefix "W"
3. API version, 1
4. int8, requested movement mode:  
    - 1 = move all joints as fast as possible  
    - 2 = synchronize joints to arrive at the destination simultaneously
    - 3 = calibration mode
    - 4 = return to standard/zero angles, and exit
5. int8, desired angle for gripper
6.  int8, desired angle for wrist  
7.  int8, desired angle for underarm 
8.  int8, desired angle for elbow   
9.  int8, desired angle for overarm  
10. int8, desired angle for shoulder_forward 
11. int8, desired angle for shoulder_out  
12. int8, last known angle for gripper
13. int8, last known angle for wrist  
14. int8, last known angle for underarm 
15. int8, last known angle for elbow   
16. int8, last known angle for overarm  
17. int8, last known angle for shoulder_forward 
18. int8, last known angle for shoulder_out  
  
Reply message from Lego Technic Hub 1 or 2 or Arduino to PC:

1. Prefix "T"
2. Prefix "W"
3. API version, 1
4. int8, information source:  
    - 10 = Lego Technic Hub 1 (lower arm), ignore shoulder and gripper joint values
    - 11 = Lego Technic Hub 2 (shoulder), ignore lower arm and gripper joint values
    - 12 = Arduino (gripper), ignore shoulder and lower arm joint values
5.  int8, error code - 0 means no error
6.  int8, current angle for gripper (or -128 if unknown)
7.  int8, current angle for wrist (or -128 if unknown)
8.  int8, current angle for underarm (or -128 if unknown)
9.  int8, current angle for elbow (or -128 if unknown)
10. int8, current angle for overarm (or -128 if unknown)
11. int8, current angle for shoulder_forward (or -128 if unknown)
12. int8, current angle for shoulder_out (or -128 if unknown)
