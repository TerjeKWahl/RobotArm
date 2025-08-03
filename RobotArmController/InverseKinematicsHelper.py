"""
This file handles inverse kinematics, calculating joint angles from 
end-effector poses (translation and orientation).
"""

import numpy as np
import threading
from typing import List
from spatialmath import SE3
from RobotMessageManager import JointAngles, DistanceAndAngleOffset, UNKNOWN_ANGLE
from Configuration import robot_arm, NEUTRAL_POSE_SE3


is_first_call = True  # Flag to print robot arm structure only once



async def __get_current_pose(last_known_angles: JointAngles) -> List[int]:
    """
    Get the current pose of the robot arm based on the last known angles.
    This function returns the angles in degrees for each joint, ensuring that
    unknown angles are set to 0.
    """
    # Create a copy of the last known angles to avoid modifying the original
    last_known_angles_copy = JointAngles(
        gripper=last_known_angles.gripper,
        wrist=last_known_angles.wrist,
        underarm=last_known_angles.underarm,
        elbow=last_known_angles.elbow,
        overarm=last_known_angles.overarm,
        shoulder_forward=last_known_angles.shoulder_forward,
        shoulder_out=last_known_angles.shoulder_out
    )
    # If any gripper angles are unknown, set them to 0
    if last_known_angles_copy.gripper == UNKNOWN_ANGLE:
        last_known_angles_copy.gripper = 0
    if last_known_angles_copy.wrist == UNKNOWN_ANGLE:
        last_known_angles_copy.wrist = 0
    if last_known_angles_copy.underarm == UNKNOWN_ANGLE:
        last_known_angles_copy.underarm = 0
    if last_known_angles_copy.elbow == UNKNOWN_ANGLE:
        last_known_angles_copy.elbow = 0
    if last_known_angles_copy.overarm == UNKNOWN_ANGLE:
        last_known_angles_copy.overarm = 0
    if last_known_angles_copy.shoulder_forward == UNKNOWN_ANGLE:
        last_known_angles_copy.shoulder_forward = 0
    if last_known_angles_copy.shoulder_out == UNKNOWN_ANGLE:
        last_known_angles_copy.shoulder_out = 0
    return [last_known_angles_copy.shoulder_forward, last_known_angles_copy.shoulder_out, last_known_angles_copy.overarm, last_known_angles_copy.elbow, last_known_angles_copy.underarm, last_known_angles_copy.wrist]



async def calculate_inverse_kinematics(last_known_angles: JointAngles, 
                                       desired_distance_and_angle_offset: DistanceAndAngleOffset) -> JointAngles: 
    """
    Calculate the inverse kinematics for the robot arm.
    Returns the desired angles for each joint based on the desired distance and angle offset.
    If the calculation fails, it returns None and prints an error message.
    """
    # Print general information about the robot arm structure only if this is the first time this function is called:
    global is_first_call
    if is_first_call:
        is_first_call = False
        print("The robot arm structure is:")
        print(robot_arm) # View the ETS (Elementary Transformation Sequence, where ET is a translation or rotation).
        print(f"The robot has {robot_arm.n} joints")
        print(f"The robot has {robot_arm.m} Elementary Transformations (ETs)")

    # Calculate the desired angles based on the desired distance and angle offset of the robot arm.
    desired_translation = SE3.Trans(
        desired_distance_and_angle_offset.x_distance,
        desired_distance_and_angle_offset.y_distance, 
        desired_distance_and_angle_offset.z_distance
    )            
    desired_rotation = SE3.RPY( # TODO: Fix/test ordering of rotations
        np.deg2rad(desired_distance_and_angle_offset.x_angle),
        np.deg2rad(desired_distance_and_angle_offset.y_angle),
        np.deg2rad(desired_distance_and_angle_offset.z_angle)
    )
    # Apply the offsets relative to the neutral pose
    desired_pose = NEUTRAL_POSE_SE3 * desired_translation * desired_rotation

    current_pose_q = await __get_current_pose(last_known_angles)
    print(f"\nDesired distance and angle offset: {desired_distance_and_angle_offset}")
    desired_angles = None

    mask_priority = np.array([4, 4, 4, 1, 2, 3])  # We want to prioritize the position over the orientation in the IK solution, 
                                                  # and for orientation we don't prioritize rotation around the X axis (should be along the gripper "fingers" so not too important).
                                                  # The Y and Z axes are more important for orientation, and we want to prioritize the Z axis (controlled by wrist) over 
                                                  # the Y axis (controlled by elbow and shoulder forward).
    #mask_priority = np.array([2, 2, 2, 0, 1, 1])  # We want to prioritize the position over the orientation in the IK solution, 
    #                                              # and for orientation we don't prioritize rotation around the X axis (should be along the gripper "fingers" so not too important).
    try:

        # Calculate inverse kinematics to get joint angles
        inverse_kinematics_solution = robot_arm.ik_LM(Tep=desired_pose, q0=current_pose_q, mask=mask_priority, joint_limits=True)

        solution_q, was_successful, iterations, num_searches, residual = inverse_kinematics_solution
        #print(f"solution_q: {solution_q}")
        joint_angles_deg = np.round(np.rad2deg(solution_q)).astype(int)
        print(f"Inverse kinematics solution {'SUCCESS' if was_successful else 'FAILED'}")
        #print(f"Termination status: {reason} (1=success, 0=failure)")
        #print(f"Solution found in {iterations} iterations")
        #print(f"Number of searches performed: {num_searches}")
        #print(f"Final residual error: {residual:.6f}")
        print(f"Solution angles (degrees): {joint_angles_deg}")
        #print(f"Rotation around the Z, Y and Z axes for the desired_pose: {desired_pose.eul(unit='deg')}")
        #solution_se3 = robot_arm.fkine(solution_q)
        #print(f"Rotation around the Z, Y and Z axes for the solution:     {solution_se3.eul(unit='deg')}")

        if was_successful:
            # Update desired angles
            desired_angles = JointAngles()
            desired_angles.shoulder_forward = joint_angles_deg[0]
            desired_angles.shoulder_out = joint_angles_deg[1]
            desired_angles.overarm = joint_angles_deg[2]
            desired_angles.elbow = joint_angles_deg[3]
            desired_angles.underarm = joint_angles_deg[4]
            desired_angles.wrist = joint_angles_deg[5]
        else:
            print("Failed to find inverse kinematics solution - use previous angles")
            
    except Exception as e:
        print(f"Error in inverse kinematics calculation: {e}")
        print("Use previous desired angles")


    # Return the desired angles
    return desired_angles

