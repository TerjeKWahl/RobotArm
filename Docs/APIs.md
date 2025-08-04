# APIs used for communicating between components

The following APIs are used:
- API for communication between PC application and Lego Technic Hub 1 (lower arm) or Lego Technic 
  Hub 2 (shoulder) or Arduino (gripper)
- API for communication between PC application and VR app


## API for communication between PC application and Lego Technic Hub 1 or 2, and Arduino:

We want to keep payloads short when communication with Lego Hubs, because of Bluetooth packet limitations (source:
https://github.com/pybricks/technical-info/blob/master/pybricks-ble-broadcast-observe.md). Because of this, a
proprietary binary communication protocol is used. Angles are in degrees. 
The numbering in the two lists below represent byte numbers.


### Message from PC to Lego Technic Hub 1 or 2 or Arduino (the same data is sent to all):

1.  Prefix "T"
2.  Prefix "W"
3.  API version, 1
4.  int8, requested movement mode:  
    - 1 = move all joints as fast as possible (for continuous tracking)
    - 2 = move to target with smooth acceleration and deceleration (run to target)
    - 3 = calibration mode
    - 4 = return to standard/zero angles, and exit
5.  int8, desired angle for gripper
6.  int8, desired angle for wrist  
7.  int8, desired angle for underarm 
8.  int8, desired angle for elbow   
9.  int8, desired angle for overarm  
10. int8, desired angle for shoulder_out  
11. int8, desired angle for shoulder_forward 
12. int8, last known angle for gripper
13. int8, last known angle for wrist  
14. int8, last known angle for underarm 
15. int8, last known angle for elbow   
16. int8, last known angle for overarm  
17. int8, last known angle for shoulder_out  
18. int8, last known angle for shoulder_forward 
  

### Message from Lego Technic Hub 1 or 2 or Arduino to PC:

1.  Prefix "T"
2.  Prefix "W"
3.  API version, 1
4.  int8, information source:  
    - 10 = Lego Technic Hub 1 (lower arm), ignore shoulder and gripper joint values
    - 11 = Lego Technic Hub 2 (shoulder), ignore lower arm and gripper joint values
    - 12 = Arduino (gripper), ignore shoulder and lower arm joint values
5.  int8, error code - 0 means no error
6.  int8, current angle for gripper (or -128 if unknown)
7.  int8, current angle for wrist (or -128 if unknown)
8.  int8, current angle for underarm (or -128 if unknown)
9.  int8, current angle for elbow (or -128 if unknown)
10. int8, current angle for overarm (or -128 if unknown)
11. int8, current angle for shoulder_out (or -128 if unknown)
12. int8, current angle for shoulder_forward (or -128 if unknown)



## API for communication between PC application and VR app

To make communication efficient on a LAN, we use UDP packets and make sure the payload is short enough 
to fit within one WiFi/Ethernet UDP package (around 1400 bytes).

When talking about position offsets:
- X is forward
- Y is to the left
- Z is up

VR app should send a UDP package to the PC every second frame (around 30 times per second).

The PC should send a UDP package to the VR app 30 times per second.

16 bit numbers are sent big-endian (MSB first, LSB last). Distances are in mm. Angles are in degrees. 
Offsets mean in relation to the initial starting position of the robot arm.


### Message from VR to PC:

1.  Prefix "T"
2.  Prefix "K"
3.  Prefix "W"
4.  API version, 1
5.  int8, information source:  
    - 20 = VR app
6.  (Reserved for future use, extra byte to avoid bugs due to struct packing.)
7.  (Reserved for future use, extra byte to avoid bugs due to struct packing.)
8.  (Reserved for future use, extra byte to avoid bugs due to struct packing.)
9.  Bytes number 9 to 136: 16 double floating point numbers, representing the 4 x 4 matrix 
    describing the position and rotation of the VR gripper.
    16 doubles * 8 bytes/double = 128 bytes.
    The numbers are row first, so the first 4 numbers represent the first row, etc.


### Message from PC to VR:

1.  Prefix "T"
2.  Prefix "K"
3.  Prefix "W"
4.  API version, 1
5.  int8, information source:  
    - 21 = PC app
6.  int8, PC connection status:
    - 0 = Not connected to controllers (Lego and Arduino)
    - 1 = Connected to controllers (Lego and Arduino)
7.  int16 MSB, X last known distance offset
8.  int16 LSB, X last known distance offset
9.  int16 MSB, Y last known distance offset
10. int16 LSB, Y last known distance offset
11. int16 MSB, Z last known distance offset
12. int16 LSB, Z last known distance offset
13. int16 MSB, X last known angle offset
14. int16 LSB, X last known angle offset
15. int16 MSB, Y last known angle offset
16. int16 LSB, Y last known angle offset
17. int16 MSB, Z last known angle offset
18. int16 LSB, Z last known angle offset
19. int8, last known angle for gripper
20. int8, last known angle for wrist  
21. int8, last known angle for underarm 
22. int8, last known angle for elbow   
23. int8, last known angle for overarm  
24. int8, last known angle for shoulder_out  
25. int8, last known angle for shoulder_forward 
