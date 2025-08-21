"""
Configuration constants for the Robot Arm Controller PC app.
"""
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from RunMode import RunMode

# Software version of this program
SW_VERSION = "0.5.0"

# Run mode to use
RUN_MODE = RunMode.VR_FOLLOWING_MODE  # Change to RunMode.VR_FOLLOWING_MODE for VR following mode, or RunMode.DEMO_MODE for demo mode

# Hub names for Bluetooth discovery
HUB_NAME_SHOULDER_CONTROLLER = "Pybricks Hub"
HUB_NAME_LOWER_ARM_CONTROLLER = "Pybricks hub 2"

VR_IP_ADDRESS = "192.168.0.47" # IP address of the VR device
VR_UDP_PORT = 7506             # Port for UDP communication with the VR device (both sending and receiving)
ARDUINO_IP_ADDRESS = "192.168.0.67" # IP address of the Arduino device
ARDUINO_UDP_PORT = 7507             # Port for UDP communication between Arduino and PC (both sending and receiving)

GRIPPER_ANGLE_MAX_DEG = 45

# Definition of the Lego robot arm structure (links and joints). This is a right arm:
studs = 0.008  # 8 mm between Lego studs
# Define joint limits in degrees
joint_limits_deg = [
    [-35, 45],    # Joint 1 (shoulder forward)
    [-10, 35],    # Joint 2 (shoulder out/up): 
    [-120, 120],  # Joint 3 (overarm):        
    [-85, 23],    # Joint 4 (elbow):          
    [-120, 120],  # Joint 5 (underarm):       
    [-110, 110],  # Joint 6 (wrist):           - Wrist down/up (as seen when right thumb is up)
    [-70, 70]     # Joint 7 (wrist other way): - This joint is not physically on the robot (yet), but greatly improves IK solutions and control stability as a "virtual" joint
]
joint_limits_rad = np.deg2rad(joint_limits_deg)
# The robot's Elementary Transformations (ETs):
E1 = rtb.ET.tz(0.377)                                 # 37.7 cm up from the desk/table
E2 = rtb.ET.Ry(flip=True, qlim=joint_limits_rad[0])   # Shoulder rotation forward/back                       (positive angle is forwards)
E3 = rtb.ET.ty(-9.5*studs)                            # 9.5 studs out to next shoulder joint
E4 = rtb.ET.Rx(flip=True, qlim=joint_limits_rad[1])   # Shoulder rotation out/up / in/down to the side       (positive angle is out/up)
E5 = rtb.ET.tz(-20.5*studs)                           # 20.5 studs down to overarm rotation joint
E6 = rtb.ET.Rz(qlim=joint_limits_rad[2])              # Overarm rotation                                     (positive angle is counter clockwise in towards the body)
E7 = rtb.ET.tz(-13.5*studs)                           # 13.5 studs down to elbow
E8 = rtb.ET.Ry(flip=True, qlim=joint_limits_rad[3])   # Elbow rotation                                       (positive angle is flexing the elbow upwards)
E9 = rtb.ET.tx(7.5*studs)                             # 7.5 studs out to underarm rotation joint
E10 = rtb.ET.Rx(qlim=joint_limits_rad[4])             # Underarm rotation                                    (positive angle is rotating the underarm clockwise)
E11 = rtb.ET.tx(22.5*studs)                           # 22.5 studs out to wrist rotation joint
E12 = rtb.ET.Ry(qlim=joint_limits_rad[5])             # Wrist rotation down/up (as seen when thumb is up)    (positive angle is rotating the wrist down)
E13 = rtb.ET.tx(3.5*studs)                            # 3.5 studs out to other wrist joint
E14 = rtb.ET.Rz(qlim=joint_limits_rad[6])             # Wrist rotation left/right                            (this is a virtual joint not actually on the robot)
E15 = rtb.ET.tx(3.5*studs)                            # 3.5 studs out to gripper joint actuator

robot_arm = E1 * E2 * E3 * E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13 * E14 * E15

# Robot initial position and orientation (neutral pose with bent elbow and thumb pointing up)
neutral_pose_with_bent_elbow_q = np.array([0, 0, 0, 0, 0, 0, 0])
NEUTRAL_POSE_SE3 = robot_arm.fkine(neutral_pose_with_bent_elbow_q)  # Calculate the forward kinematics of the neutral pose with a bent elbow

unity_position_offset = np.array([0.25, -0.40, 0.30])    # Offset in meters (in Unity starts 25 cm right, 
                                                         # 40 cm down from eye level and 30 cm forward)
robot_position_offset = np.array([0.296, -0.076, 0.105]) # Offset in meters (physical robot arm starts 29.6 cm forward, 
                                                         # 7.6 cm right and 11.3 cm up from table level)
