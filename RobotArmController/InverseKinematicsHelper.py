"""
This file handles inverse kinematics, calculating joint angles from 
end-effector poses (translation and orientation).
"""

import numpy as np
import roboticstoolbox as rtb
from typing import List
from spatialmath import SE3
from RobotMessageManager import JointAngles, Matrix4x4, UNKNOWN_ANGLE
from Configuration import robot_arm, joint_limits_deg, unity_position_offset, robot_position_offset, NEUTRAL_POSE_SE3


is_first_call = True  # Flag to print robot arm structure only once

# Definition of rotation matrix for conversion from 
# Unity's coordinate system (x-right, y-up, z-forward)
# to the robot arm's coordinate system (x-forward, y-left, z-up):
unity_to_robot_rotation = np.array([
    [0,  0, 1, 0],   # Unity z (forward) → Robot x (forward)
    [-1, 0, 0, 0],   # Unity x (right) → Robot -y (left)
    [0,  1, 0, 0],   # Unity y (up) → Robot z (up)
    [0,  0, 0, 1]     
]).astype(np.float64)  # Use float64 for higher precision
unity_to_robot_rotation_inv = np.linalg.inv(unity_to_robot_rotation)

last_desired_7th_joint_angle_deg = 0 # Temporary(?) variable representing the virtual 7th joint (2nd wrist joint) in the robot



def __get_current_pose_deg(last_known_angles: JointAngles) -> List[int]:
    """
    Get the current pose of the robot arm based on the last known angles.
    This function returns the angles in degrees for each joint, ensuring that
    unknown angles are set to 0.
    """
    global last_desired_7th_joint_angle_deg
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
    
    return [last_known_angles_copy.shoulder_forward, last_known_angles_copy.shoulder_out, last_known_angles_copy.overarm, 
            last_known_angles_copy.elbow, last_known_angles_copy.underarm, last_known_angles_copy.wrist,
            last_desired_7th_joint_angle_deg]


def __get_pos_in_robot_coordinate_system_se3(vr_unity_matrix_4x4: Matrix4x4) -> SE3:
    """
    Convert a 4x4 matrix from Unity's coordinate system to the robot arm's coordinate system.
    If an error occurs during the conversion, it returns a representation of the robot's neutral pose.
    """
    global unity_to_robot_rotation, unity_to_robot_rotation_inv, unity_position_offset, robot_position_offset
    
    #print(f"unity_to_robot_rotation: \n{unity_to_robot_rotation}")
    #print(f"unity_to_robot_rotation_inv: \n{unity_to_robot_rotation_inv}")
    #print(f"unity_to_robot_rotation_inv has type: {unity_to_robot_rotation_inv.dtype}")
    m = vr_unity_matrix_4x4
    unity_pos_array_4x4 = np.array([[m.m00, m.m01, m.m02, m.m03],
                                    [m.m10, m.m11, m.m12, m.m13],
                                    [m.m20, m.m21, m.m22, m.m23],
                                    [m.m30, m.m31, m.m32, m.m33]]).astype(np.float64)  # Use float64 for higher precision
    #print(f"unity_pos_array_4x4: \n{unity_pos_array_4x4}")
    translated_unity_pos = unity_pos_array_4x4
    translated_unity_pos[:3, 3] -= unity_position_offset  # Subtract the Unity offset to the position
    #print(f"translated_unity_pos: \n{translated_unity_pos}")
    pos_in_robot_coordinate_system = np.dot( np.dot(unity_to_robot_rotation, unity_pos_array_4x4), unity_to_robot_rotation_inv)
    #print(f"pos_in_robot_coordinate_system (before rotational component fix): \n{pos_in_robot_coordinate_system}")

    # Extract rotation and translation parts
    rotation_part = pos_in_robot_coordinate_system[:3, :3]
    translation_part = pos_in_robot_coordinate_system[:3, 3]

    #print(f"Determinant of rotation part before adjusting: {np.linalg.det(rotation_part)}")
    # Normalize the rotation matrix using SVD
    U, s, Vt = np.linalg.svd(rotation_part)
    rotation_normalized = U @ Vt
    #print(f"Determinant of rotation part after adjusting: {np.linalg.det(rotation_normalized)}")
    # Ensure proper rotation (det = 1, not -1)
    if np.linalg.det(rotation_normalized) < 0:
        Vt[-1, :] *= -1
        rotation_normalized = U @ Vt
    #print(f"Determinant of rotation part after adjusting and ensuring proper rotation: {np.linalg.det(rotation_normalized)}")
    # Reconstruct the transformation matrix
    pos_in_robot_coordinate_system_fixed = np.eye(4)
    pos_in_robot_coordinate_system_fixed[:3, :3] = rotation_normalized
    pos_in_robot_coordinate_system_fixed[:3, 3] = translation_part
    #print(f"pos_in_robot_coordinate_system_fixed: \n{pos_in_robot_coordinate_system_fixed}")
    pos_in_robot_coordinate_system = pos_in_robot_coordinate_system_fixed

    pos_in_robot_coordinate_system_se3 = SE3(NEUTRAL_POSE_SE3)
    try:
        pos_in_robot_coordinate_system_se3 = SE3(pos_in_robot_coordinate_system)
        #print(f"pos_in_robot_coordinate_system_se3 (before applying offset): \n{pos_in_robot_coordinate_system_se3}")
        pos_in_robot_coordinate_system[:3, 3] += robot_position_offset  # Add the robot coordinate system offset to the position
        pos_in_robot_coordinate_system_se3 = SE3(pos_in_robot_coordinate_system)
        #print(f"pos_in_robot_coordinate_system_se3 (after applying offset): \n{pos_in_robot_coordinate_system_se3}")
    except Exception as e:
        print(f"Error creating SE3 from pos_in_robot_coordinate_system: {e}")
        print(f"pos_in_robot_coordinate_system has shape: {pos_in_robot_coordinate_system.shape}")
        print(f"pos_in_robot_coordinate_system dtype: {pos_in_robot_coordinate_system.dtype}")
        rotation_part = pos_in_robot_coordinate_system[:3, :3]
        print(f"Determinant: {np.linalg.det(rotation_part)}")

    return pos_in_robot_coordinate_system_se3



def __transform_to_vr_coordinate_system(robot_pose_se3: SE3) -> np.ndarray:
    """
    Transform a pose from the robot's coordinate system back to Unity's VR coordinate system.
    This is the inverse of the transformation done in __get_pos_in_robot_coordinate_system_se3.
    
    :param robot_pose_se3: SE3 pose in robot coordinate system
    :return: 4x4 transformation matrix in Unity/VR coordinate system
    """
    global unity_to_robot_rotation, unity_to_robot_rotation_inv, unity_position_offset, robot_position_offset
    
    # Get the 4x4 matrix from SE3
    robot_matrix = robot_pose_se3.A
    
    # Remove the robot position offset
    robot_matrix_without_offset = robot_matrix.copy()
    robot_matrix_without_offset[:3, 3] -= robot_position_offset
    
    # Transform back to Unity coordinate system (inverse of the original transformation)
    unity_matrix = np.dot(np.dot(unity_to_robot_rotation_inv, robot_matrix_without_offset), unity_to_robot_rotation)
    
    # Add back the Unity position offset
    unity_matrix[:3, 3] += unity_position_offset
    
    return unity_matrix



def get_VR_position_and_orientation_matrix_from_last_known_angles(last_known_angles: JointAngles) -> Matrix4x4: 
    """
    Get the last known position and orientation matrix for the physical robot arm (in Unity/VR coordinate system) 
    from the last known angles.
    Uses forward kinematics to compute the end-effector pose. Used to send to VR app, so it can display 
    a "ghost" of the real robot arm.
    """
    global last_desired_7th_joint_angle_deg

    q = last_known_angles.as_np_array_rad(last_desired_7th_joint_angle_deg)
    pose_se3 = robot_arm.fkine(q) # Calculate the forward kinematics to get the end-effector pose

    # Transform the position and orientation into the VR/Unity coordinate system
    pose_transformed_to_vr_coordinate_system = __transform_to_vr_coordinate_system(pose_se3)

    matrix_4x4 = Matrix4x4()
    matrix_4x4.m00 = pose_transformed_to_vr_coordinate_system[0, 0]
    matrix_4x4.m01 = pose_transformed_to_vr_coordinate_system[0, 1]
    matrix_4x4.m02 = pose_transformed_to_vr_coordinate_system[0, 2]
    matrix_4x4.m03 = pose_transformed_to_vr_coordinate_system[0, 3]
    matrix_4x4.m10 = pose_transformed_to_vr_coordinate_system[1, 0]
    matrix_4x4.m11 = pose_transformed_to_vr_coordinate_system[1, 1]
    matrix_4x4.m12 = pose_transformed_to_vr_coordinate_system[1, 2]
    matrix_4x4.m13 = pose_transformed_to_vr_coordinate_system[1, 3]
    matrix_4x4.m20 = pose_transformed_to_vr_coordinate_system[2, 0]
    matrix_4x4.m21 = pose_transformed_to_vr_coordinate_system[2, 1]
    matrix_4x4.m22 = pose_transformed_to_vr_coordinate_system[2, 2]
    matrix_4x4.m23 = pose_transformed_to_vr_coordinate_system[2, 3]
    matrix_4x4.m30 = 0
    matrix_4x4.m31 = 0
    matrix_4x4.m32 = 0
    matrix_4x4.m33 = 1
    return matrix_4x4



def __adjust_angles_to_not_crash_into_anything(desired_angles : JointAngles):
    """ Adjust angles to not crash into the table/desk or the robot torso. """
    global last_desired_7th_joint_angle_deg

    MIN_HEIGHT_OVER_TABLE_M = 0.00
    MIN_Y_POSITION_WRT_TORSO_M = -0.025
    ANGLE_ADJUSTMENT_STEP_DEG = 2

    # First clip desired angles to valid range based on simple limits, in case it is not already done:
    desired_angles.gripper = max(-128, min(127, desired_angles.gripper))
    desired_angles.wrist = max(joint_limits_deg[5][0], min(joint_limits_deg[5][1], desired_angles.wrist))
    desired_angles.underarm = max(joint_limits_deg[4][0], min(joint_limits_deg[4][1], desired_angles.underarm))
    desired_angles.elbow = max(joint_limits_deg[3][0], min(joint_limits_deg[3][1], desired_angles.elbow))
    desired_angles.overarm = max(joint_limits_deg[2][0], min(joint_limits_deg[2][1], desired_angles.overarm))
    desired_angles.shoulder_out = max(joint_limits_deg[1][0], min(joint_limits_deg[1][1], desired_angles.shoulder_out))
    desired_angles.shoulder_forward = max(joint_limits_deg[0][0], min(joint_limits_deg[0][1], desired_angles.shoulder_forward))

    # Get SE3 pose from the desired angles
    desired_pose_matrix = robot_arm.fkine(desired_angles.as_np_array_rad(last_desired_7th_joint_angle_deg)).A

    # First make sure we don't crash into the table/desk:
    while desired_pose_matrix[2, 3] < MIN_HEIGHT_OVER_TABLE_M:
        # If the end-effector is too low, raise it by adjusting the elbow joint (if not already too high), alternatively the shoulder joint
        if desired_angles.elbow <= joint_limits_deg[3][1] + ANGLE_ADJUSTMENT_STEP_DEG:
            desired_angles.elbow += ANGLE_ADJUSTMENT_STEP_DEG
        elif desired_angles.shoulder_forward <= joint_limits_deg[0][1] + ANGLE_ADJUSTMENT_STEP_DEG:
            desired_angles.shoulder_forward += ANGLE_ADJUSTMENT_STEP_DEG
        else:
            print("ERROR: Cannot raise the end-effector to get above the table. This should not be possible.")
        desired_pose_matrix = robot_arm.fkine(desired_angles.as_np_array_rad(last_desired_7th_joint_angle_deg)).A

    # Now make sure we don't crash into the robot's torso:
    # TODO Make this more fancy so the robot arm can reach in front of the torso if the shoulder out joint is extended enough
    while desired_pose_matrix[1, 3] > MIN_Y_POSITION_WRT_TORSO_M:  # If the end-effector is too close to the torso
        if desired_angles.overarm >= joint_limits_deg[2][0] - ANGLE_ADJUSTMENT_STEP_DEG:
            desired_angles.overarm -= ANGLE_ADJUSTMENT_STEP_DEG
        else:
            print("ERROR: Cannot move the end-effector away from the torso. This should not be possible.")
        desired_pose_matrix = robot_arm.fkine(desired_angles.as_np_array_rad(last_desired_7th_joint_angle_deg)).A

    return desired_angles



def get_desired_angles_from_VR_position_and_orientation_matrix(last_known_angles: JointAngles, 
                                                               vr_unity_matrix_4x4: Matrix4x4) -> JointAngles: 
    """
    Calculate the inverse kinematics for the robot arm.
    Returns the desired angles for each joint based on the desired position and orientation.

    :param last_known_angles: The last known angles of the robot arm joints. Used to get IK solutions close to the last known angles.
    :param vr_unity_matrix_4x4: The 4x4 matrix representing the desired position and orientation in Unity world coordinates
    :return: JointAngles object with the desired angles for each joint, in degrees. If the calculation fails,
             it returns None and prints an error message.
    """
    global is_first_call, last_desired_7th_joint_angle_deg

    # Print general information about the robot arm structure only if this is the first time this function is called:
    if is_first_call:
        is_first_call = False
        print("The robot arm structure is:")
        print(robot_arm) # View the ETS (Elementary Transformation Sequence, where ET is a translation or rotation).
        print(f"The robot has {robot_arm.n} joints")
        print(f"The robot has {robot_arm.m} Elementary Transformations (ETs)")

        # TODO: On first call, also make alternative robot arms with stricter limits for the virtual second wrist joint, and 
        # use this to find IK solutions that minimize the virtual wrist joint angle, to minimize errors between virtual and actual robot arm.


    desired_pose_se3 = __get_pos_in_robot_coordinate_system_se3(vr_unity_matrix_4x4)
    #print(f"desired_pose_se3: \n{desired_pose_se3}")
    current_pose_q = np.deg2rad(__get_current_pose_deg(last_known_angles))
    desired_angles = None

    mask_priority_1 = np.array([4, 4, 4, 2, 1, 3])  # We want to prioritize the position over the orientation in the IK solution, 
                                                    # and for orientation we don't prioritize rotation around the X axis (should be along the gripper "fingers" so not too important).
                                                    # The X and Z axes are more important for orientation, and we want to prioritize the Z axis (controlled by wrist) over 
                                                    # the X axis (controlled by underarm rotation).
    mask_priority_2 = np.array([2, 2, 2, 1, 0, 1])  # We want to prioritize the position over the orientation in the IK solution, 
                                                    # and for orientation we don't prioritize rotation around the Y axis (missing wrist joint around 2nd axis). TODO change and test
    mask_list = [mask_priority_1, mask_priority_2]  # List of masks to try

    joint_angles_deg = 0
    was_successful = False
    # TODO for robot arms in robot arm list
    for mask_priority in mask_list:
        #print(f"\n\nTrying mask priority: {mask_priority} for desired pose:")

        # Solve IK (inverse kinematics):
        joint_angles_solution = robot_arm.ik_LM(Tep=desired_pose_se3, q0=current_pose_q, mask=mask_priority, joint_limits=True, ilimit=30, slimit=100) # TODO: Takes long time? Consider having slimit lower except for the most liberal robot arm representation.
        solution_q, was_successful, iterations, num_searches, residual = joint_angles_solution
        #print(f"LM Inverse kinematics solution: {'SUCCESS' if was_successful else 'FAILURE'}", end='. ')
        if was_successful:
            joint_angles_deg = np.round(np.rad2deg(solution_q)).astype(int)
            #print(f"Solution found in {iterations} iterations")
            #print(f"Number of searches performed: {num_searches}")
            #print(f"Final residual error: {residual:.6f}")
            #print(f"Joint angles suggested (degrees): {joint_angles_deg}")
            #print(f"Rotation around the Z, Y and Z axes for the desired_pose_se3: {desired_pose_se3.eul(unit='deg')}")
            #solution_se3 = robot_arm.fkine(solution_q)
            #print(f"Rotation around the Z, Y and Z axes for the solution:    {solution_se3.eul(unit='deg')}")
            break  # If the solution was successful, no need to try other methods
        else:
            print(f"***** Inverse kinematics failed for mask {mask_priority}. *****")

    if not was_successful:
        print("!!!!! WARNING !!!!!! Failed to find inverse kinematics solution - use previous angles")
    else:

        # Update desired angles
        desired_angles = JointAngles()
        desired_angles.shoulder_forward = joint_angles_deg[0]
        desired_angles.shoulder_out = joint_angles_deg[1]
        desired_angles.overarm = joint_angles_deg[2]
        desired_angles.elbow = joint_angles_deg[3]
        desired_angles.underarm = joint_angles_deg[4]
        desired_angles.wrist = joint_angles_deg[5]
        last_desired_7th_joint_angle_deg = joint_angles_deg[6] # TODO: This may be temporary, but improves stability

        __adjust_angles_to_not_crash_into_anything(desired_angles) # Adjust angles to not crash the arm into the table/desk, the robot torso or itself

    # Return the desired angles
    return desired_angles
