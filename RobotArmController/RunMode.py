"""
Run mode enumeration for the Robot Arm Controller.
"""

from enum import Enum


class RunMode(Enum):
    """Defines the different operating modes for the robot arm."""
    DEMO_MODE = "demo"
    VR_FOLLOWING_MODE = "vr_following"
