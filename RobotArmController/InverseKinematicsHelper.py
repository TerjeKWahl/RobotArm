"""
This file handles inverse kinematics, calculating joint angles from 
end-effector poses (translation and orientation).
"""

import numpy as np
import threading
from typing import List
from spatialmath import SE3
from RobotMessageManager import JointAngles, DistanceAndAngleOffset, Matrix4x4, UNKNOWN_ANGLE
from Configuration import robot_arm, joint_limits_deg, NEUTRAL_POSE_SE3


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
# TODO: Move these to Configuration.py 
unity_postion_offset = np.array([0.25, -0.40, 0.30])    # Offset in meters (in Unity starts 25cm right, 
                                                        # 40cm down from eye level and 30cm forward)
robot_postion_offset = np.array([0.328, -0.076, 0.113]) # Offset in meters (physical robot arm starts 32.8cm forward, 
                                                        # 7,6cm right and 11.3cm up from table level)


def __get_current_pose(last_known_angles: JointAngles) -> List[int]:
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



async def calculate_distance_and_angle_offset(joint_angles : JointAngles) -> DistanceAndAngleOffset:
    """
    Calculate the distance and angle offset for the robot arm based on the last known angles.
    """
    """ Test code for later:
    q = np.deg2rad(np.array([0,0,0,0,0,0]))  # Initialize with zeros
    position_and_angle_se3 = robot_arm.fkine(q) # Calculate the forward kinematics to get the end-effector pose
    xyz_angles = position_and_angle_se3.eul(unit='deg') # Get rotation around the Z, Y and Z axes for the solution. TODO: Fix/test correctness of ordering of rotations (print out many alternatives to find the right one)
    dist_and_angles = DistanceAndAngleOffset(
        x_distance = int(position_and_angle_se3.t[0]),
        y_distance = int(position_and_angle_se3.t[1]),
        z_distance = int(position_and_angle_se3.t[2]),
        x_angle = int(xyz_angles[0]),
        y_angle = int(xyz_angles[1]),
        z_angle = int(xyz_angles[2])
    )
    print(f"Calculated last known distance and angle offset zeroes: {dist_and_angles}")

    q = np.deg2rad(np.array([0,0,0,0,90,0]))  # Initialize with zeros
    position_and_angle_se3 = robot_arm.fkine(q) # Calculate the forward kinematics to get the end-effector pose
    xyz_angles = position_and_angle_se3.eul(unit='deg') # Get rotation around the Z, Y and Z axes for the solution. TODO: Fix/test correctness of ordering of rotations (print out many alternatives to find the right one)
    dist_and_angles = DistanceAndAngleOffset(
        x_distance = int(position_and_angle_se3.t[0]),
        y_distance = int(position_and_angle_se3.t[1]),
        z_distance = int(position_and_angle_se3.t[2]),
        x_angle = int(xyz_angles[0]),
        y_angle = int(xyz_angles[1]),
        z_angle = int(xyz_angles[2])
    )
    print(f"Calculated last known distance and angle offset underarm 90: {dist_and_angles}")

    q = np.deg2rad(np.array([0,0,0,0,0,90]))  # Initialize with zeros
    position_and_angle_se3 = robot_arm.fkine(q) # Calculate the forward kinematics to get the end-effector pose
    xyz_angles = position_and_angle_se3.eul(unit='deg') # Get rotation around the Z, Y and Z axes for the solution. TODO: Fix/test correctness of ordering of rotations (print out many alternatives to find the right one)
    dist_and_angles = DistanceAndAngleOffset(
        x_distance = int(position_and_angle_se3.t[0]),
        y_distance = int(position_and_angle_se3.t[1]),
        z_distance = int(position_and_angle_se3.t[2]),
        x_angle = int(xyz_angles[0]),
        y_angle = int(xyz_angles[1]),
        z_angle = int(xyz_angles[2])
    )
    print(f"Calculated last known distance and angle offset wrist 90: {dist_and_angles}")

    q = np.deg2rad(np.array([0,0,0,90,0,0]))  # Initialize with zeros
    position_and_angle_se3 = robot_arm.fkine(q) # Calculate the forward kinematics to get the end-effector pose
    xyz_angles = position_and_angle_se3.eul(unit='deg') # Get rotation around the Z, Y and Z axes for the solution. TODO: Fix/test correctness of ordering of rotations (print out many alternatives to find the right one)
    dist_and_angles = DistanceAndAngleOffset(
        x_distance = int(position_and_angle_se3.t[0]),
        y_distance = int(position_and_angle_se3.t[1]),
        z_distance = int(position_and_angle_se3.t[2]),
        x_angle = int(xyz_angles[0]),
        y_angle = int(xyz_angles[1]),
        z_angle = int(xyz_angles[2])
    )
    print(f"Calculated last known distance and angle offset elbow 90: {dist_and_angles}")
    """


    q = np.deg2rad(np.array([
        joint_angles.shoulder_forward,
        joint_angles.shoulder_out,
        joint_angles.overarm,
        joint_angles.elbow,
        joint_angles.underarm,
        joint_angles.wrist
    ]))
    position_and_angle_se3 = robot_arm.fkine(q) # Calculate the forward kinematics to get the end-effector pose
    xyz_angles = position_and_angle_se3.eul(unit='deg') # Get rotation around the Z, Y and Z axes for the solution. TODO: Fix/test correctness of ordering of rotations (print out many alternatives to find the right one)
    return DistanceAndAngleOffset(
        x_distance = int(position_and_angle_se3.t[0]),
        y_distance = int(position_and_angle_se3.t[1]),
        z_distance = int(position_and_angle_se3.t[2]),
        x_angle = int(xyz_angles[0]),
        y_angle = int(xyz_angles[1]),
        z_angle = int(xyz_angles[2])
    )



def get_desired_angles_from_VR_position_and_orientation_matrix(last_known_angles: JointAngles, 
                                                               vr_unity_matrix_4x4: Matrix4x4) -> JointAngles: 
    """
    Calculate the inverse kinematics for the robot arm.
    Returns the desired angles for each joint based on the desired position and orientation.

    :param last_known_angles: The last known angles of the robot arm joints
    :param vr_unity_matrix_4x4: The 4x4 matrix representing the desired position and orientation in Unity world coordinates
    :return: JointAngles object with the desired angles for each joint, in degrees. If the calculation fails,
             it returns None and prints an error message.
    """
    # Print general information about the robot arm structure only if this is the first time this function is called:
    global is_first_call
    if is_first_call:
        is_first_call = False
        print("The robot arm structure is:")
        print(robot_arm) # View the ETS (Elementary Transformation Sequence, where ET is a translation or rotation).
        print(f"The robot has {robot_arm.n} joints")
        print(f"The robot has {robot_arm.m} Elementary Transformations (ETs)")

    # Calculate the desired angles based on the desired distance and orientation offset of the robot arm.

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
    translated_unity_pos[:3, 3] -= unity_postion_offset  # Subtract the Unity offset to the position
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

    try:
        pos_in_robot_coordinate_system_se3 = SE3(pos_in_robot_coordinate_system)
        #print(f"pos_in_robot_coordinate_system_se3 (before applying offset): \n{pos_in_robot_coordinate_system_se3}")
        pos_in_robot_coordinate_system[:3, 3] += robot_postion_offset  # Add the robot coordinate system offset to the position
        pos_in_robot_coordinate_system_se3 = SE3(pos_in_robot_coordinate_system)
        #print(f"pos_in_robot_coordinate_system_se3 (after applying offset): \n{pos_in_robot_coordinate_system_se3}")
    except Exception as e:
        print(f"Error creating SE3 from pos_in_robot_coordinate_system: {e}")
        print(f"pos_in_robot_coordinate_system has shape: {pos_in_robot_coordinate_system.shape}")
        print(f"pos_in_robot_coordinate_system dtype: {pos_in_robot_coordinate_system.dtype}")
        rotation_part = pos_in_robot_coordinate_system[:3, :3]
        print(f"Determinant: {np.linalg.det(rotation_part)}")
        pos_in_robot_coordinate_system_se3 = SE3(NEUTRAL_POSE_SE3)

    desired_pose_se3 = pos_in_robot_coordinate_system_se3
    #print(f"desired_pose_se3: \n{desired_pose_se3}")
    current_pose_q = __get_current_pose(last_known_angles)
    desired_angles = None

    #print("Skipping inverse kinematics for now, please ignore the following error message:")
    #return desired_angles

    mask_priority_1 = np.array([4, 4, 4, 1, 2, 3])  # We want to prioritize the position over the orientation in the IK solution, 
                                                    # and for orientation we don't prioritize rotation around the X axis (should be along the gripper "fingers" so not too important).
                                                    # The Y and Z axes are more important for orientation, and we want to prioritize the Z axis (controlled by wrist) over 
                                                    # the Y axis (controlled by elbow and shoulder forward).
    mask_priority_2 = np.array([2, 2, 2, 0, 1, 1])  # We want to prioritize the position over the orientation in the IK solution, 
                                                    # and for orientation we don't prioritize rotation around the X axis (should be along the gripper "fingers" so not too important).
    mask_list = [mask_priority_1, mask_priority_2]  # List of masks to try

    joint_angles_deg = 0
    was_successful = False
    for mask_priority in mask_list:
        #print(f"\n\nTrying mask priority: {mask_priority} for desired pose:")

        # Solve IK (inverse kinematics):
        joint_angles_solution = robot_arm.ik_LM(Tep=desired_pose_se3, q0=current_pose_q, mask=mask_priority, joint_limits=True, ilimit=30, slimit=100)
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

        """
        joint_angles_solution = robot_arm.ik_NR(Tep=desired_pose_se3, q0=current_pose_q, mask=mask_priority, joint_limits=True, ilimit=30, slimit=100, pinv=True)
        solution_q, was_successful, iterations, num_searches, residual = joint_angles_solution
        print(f"NR Inverse kinematics solution: {'SUCCESS' if was_successful else 'FAILURE'}")
        if was_successful:
            joint_angles_deg = np.round(np.rad2deg(solution_q)).astype(int)
            print(f"Joint angles suggested (degrees): {joint_angles_deg}")
            break  # If the solution was successful, no need to try other methods

        joint_angles_solution = robot_arm.ik_GN(Tep=desired_pose_se3, q0=current_pose_q, mask=mask_priority, joint_limits=True, ilimit=30, slimit=100, pinv=True)
        solution_q, was_successful, iterations, num_searches, residual = joint_angles_solution
        print(f"GN Inverse kinematics solution: {'SUCCESS' if was_successful else 'FAILURE'}")
        if was_successful:
            joint_angles_deg = np.round(np.rad2deg(solution_q)).astype(int)
            print(f"Joint angles suggested (degrees): {joint_angles_deg}")
            break  # If the solution was successful, no need to try other methods
        """

    if not was_successful:
        print("!!!!! WARNING !!!!!! Failed to find inverse kinematics solution - use previous angles")
    else:

        # Check if solution respects joint limits
        q_deg = np.rad2deg(solution_q)
        within_limits = True
        for i, (q_val, limits) in enumerate(zip(q_deg, joint_limits_deg)):
            if q_val < limits[0] or q_val > limits[1]:
                print(f"WARNING: Joint {i+1} angle {q_val:.1f}° exceeds limits [{limits[0]}, {limits[1]}]")
                within_limits = False
        if not within_limits:
            print("Inverse kinematics solution exceeds joint limits - use previous angles instead.")
            desired_angles = None
        else:

            # Update desired angles
            desired_angles = JointAngles()
            desired_angles.shoulder_forward = joint_angles_deg[0]
            desired_angles.shoulder_out = joint_angles_deg[1]
            desired_angles.overarm = joint_angles_deg[2]
            desired_angles.elbow = joint_angles_deg[3]
            desired_angles.underarm = joint_angles_deg[4]
            desired_angles.wrist = joint_angles_deg[5]
            

    # Return the desired angles
    return desired_angles
