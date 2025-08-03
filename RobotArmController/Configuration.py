"""
Configuration constants for the Robot Arm Controller PC app.
"""
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from RunMode import RunMode

# Software version of this program
SW_VERSION = "0.3.0"

# Run mode to use
RUN_MODE = RunMode.VR_FOLLOWING_MODE  # Change to RunMode.VR_FOLLOWING_MODE for VR following mode, or RunMode.DEMO_MODE for demo mode

# Hub names for Bluetooth discovery
HUB_NAME_SHOULDER_CONTROLLER = "Pybricks Hub"
HUB_NAME_LOWER_ARM_CONTROLLER = "Pybricks hub 2"


# Definition of the Lego robot arm structure (links and joints). This is a right arm:
studs = 0.008  # 8 mm between Lego studs
# Define joint limits in degrees
joint_limits_deg = [ # TODO: Adjust these
    [-10, 45],    # Joint 1 (shoulder forward): +45 to -10 degrees
    [0, 35],      # Joint 2 (shoulder out/up):  +35 to 0 degrees
    [-120, 35],   # Joint 3 (overarm):          +35 to -120 degrees
    [-20, 13],    # Joint 4 (elbow):            +13 to -20 degrees
    [-120, 120],  # Joint 5 (underarm):         +120 to -120 degrees
    [-110, 110]   # Joint 6 (wrist):            +110 to -110 degrees
]
joint_limits_rad = np.deg2rad(joint_limits_deg)
# The robot's Elementary Transformations (ETs):
E1 = rtb.ET.tz(0.377)                                 # 37.7 cm up from the desk/table
E2 = rtb.ET.Ry(flip=True, qlim=joint_limits_rad[0])   # Shoulder rotation forward/back                 (positive angle is forwards)
E3 = rtb.ET.ty(-9.5*studs)                            # 9.5 studs out to next shoulder joint
E4 = rtb.ET.Rx(flip=True, qlim=joint_limits_rad[1])   # Shoulder rotation out/up / in/down to the side (positive angle is out/up)
E5 = rtb.ET.tz(-20.5*studs)                           # 20.5 studs down to overarm rotation joint
E6 = rtb.ET.Rz(qlim=joint_limits_rad[2])              # Overarm rotation                               (positive angle is counter clockwise in towards the body)
E7 = rtb.ET.tz(-12.5*studs)                           # 12.5 studs down to elbow
E8 = rtb.ET.Ry(flip=True, qlim=joint_limits_rad[3])   # Elbow rotation                                 (positive angle is flexing the elbow upwards)
E9 = rtb.ET.tx(10.5*studs)                            # 10.5 studs out to underarm rotation joint
E10 = rtb.ET.Rx(qlim=joint_limits_rad[4])             # Underarm rotation                              (positive angle is rotating the underarm clockwise)
E11 = rtb.ET.tx(23.5*studs)                           # 23.5 studs out to wrist rotation joint
E12 = rtb.ET.Rz(qlim=joint_limits_rad[5])             # Wrist rotation up/down                         (positive angle is rotating the wrist downwards/inwards)
E13 = rtb.ET.tx(7*studs)                              # 7 studs out to gripper (TODO: Fix actual length))
robot_arm = E1 * E2 * E3 * E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 * E13

# Robot initial position and orientation
neutral_pose_with_bent_elbow_q = np.array([0, 0, 0, 0, 0, 0])
NEUTRAL_POSE_SE3 = robot_arm.fkine(neutral_pose_with_bent_elbow_q)  # Calculate the forward kinematics of the neutral pose with a bent elbow

