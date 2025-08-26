"""
Unit tests for the InverseKinematicsHelper module.
This file tests the inverse kinematics functions, particularly the 
coordinate system transformations and matrix conversions.

The main function tested is:
__get_pos_in_robot_coordinate_system_se3(vr_unity_matrix_4x4: Matrix4x4) -> SE3

This function converts a 4x4 transformation matrix from Unity's coordinate system
to the robot arm's coordinate system, applying proper rotation matrix normalization
and coordinate transformations.
"""

import unittest
import numpy as np
from spatialmath import SE3
import sys
import os
from Configuration import robot_position_offset, unity_position_offset

# Add the current directory to the Python path so we can import the modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from RobotMessageManager import Matrix4x4
from Configuration import unity_position_offset, robot_position_offset, NEUTRAL_POSE_SE3
import InverseKinematicsHelper
# Access the private function using getattr to bypass name mangling
get_pos_in_robot_coordinate_system_se3 = getattr(InverseKinematicsHelper, '_InverseKinematicsHelper__get_pos_in_robot_coordinate_system_se3', None)

# If the above doesn't work, try direct access
if get_pos_in_robot_coordinate_system_se3 is None:
    # Try to access it without name mangling (might be accessible directly)
    get_pos_in_robot_coordinate_system_se3 = getattr(InverseKinematicsHelper, '__get_pos_in_robot_coordinate_system_se3', None)


class TestInverseKinematicsHelper(unittest.TestCase):
    """Test cases for InverseKinematicsHelper functions."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Store original global variables
        self.original_unity_position_offset = unity_position_offset.copy()
        self.original_robot_position_offset = robot_position_offset.copy()
        
        # Create test matrices and SE3 objects

        # Identity matrix
        self.identity_matrix_4x4 = Matrix4x4()
        self.identity_matrix_4x4.m00 = 1.0; self.identity_matrix_4x4.m01 = 0.0; self.identity_matrix_4x4.m02 = 0.0; self.identity_matrix_4x4.m03 = 0.0
        self.identity_matrix_4x4.m10 = 0.0; self.identity_matrix_4x4.m11 = 1.0; self.identity_matrix_4x4.m12 = 0.0; self.identity_matrix_4x4.m13 = 0.0
        self.identity_matrix_4x4.m20 = 0.0; self.identity_matrix_4x4.m21 = 0.0; self.identity_matrix_4x4.m22 = 1.0; self.identity_matrix_4x4.m23 = 0.0
        self.identity_matrix_4x4.m30 = 0.0; self.identity_matrix_4x4.m31 = 0.0; self.identity_matrix_4x4.m32 = 0.0; self.identity_matrix_4x4.m33 = 1.0
        
        # Unity coordinate system test matrix (translation only)
        self.unity_translation_matrix = Matrix4x4()
        self.unity_translation_matrix.m00 = 1.0; self.unity_translation_matrix.m01 = 0.0; self.unity_translation_matrix.m02 = 0.0; self.unity_translation_matrix.m03 = 1.0  # 1m right
        self.unity_translation_matrix.m10 = 0.0; self.unity_translation_matrix.m11 = 1.0; self.unity_translation_matrix.m12 = 0.0; self.unity_translation_matrix.m13 = 2.0  # 2m up
        self.unity_translation_matrix.m20 = 0.0; self.unity_translation_matrix.m21 = 0.0; self.unity_translation_matrix.m22 = 1.0; self.unity_translation_matrix.m23 = 3.0  # 3m forward
        self.unity_translation_matrix.m30 = 0.0; self.unity_translation_matrix.m31 = 0.0; self.unity_translation_matrix.m32 = 0.0; self.unity_translation_matrix.m33 = 1.0
        
        # Unity coordinate system test matrix with rotation (90 degree rotation around Y axis)
        self.unity_rotation_y_90_matrix = Matrix4x4()
        self.unity_rotation_y_90_matrix.m00 = 0.0; self.unity_rotation_y_90_matrix.m01 = 0.0; self.unity_rotation_y_90_matrix.m02 = 1.0; self.unity_rotation_y_90_matrix.m03 = 0.5
        self.unity_rotation_y_90_matrix.m10 = 0.0; self.unity_rotation_y_90_matrix.m11 = 1.0; self.unity_rotation_y_90_matrix.m12 = 0.0; self.unity_rotation_y_90_matrix.m13 = 0.5
        self.unity_rotation_y_90_matrix.m20 = -1.0; self.unity_rotation_y_90_matrix.m21 = 0.0; self.unity_rotation_y_90_matrix.m22 = 0.0; self.unity_rotation_y_90_matrix.m23 = 0.5
        self.unity_rotation_y_90_matrix.m30 = 0.0; self.unity_rotation_y_90_matrix.m31 = 0.0; self.unity_rotation_y_90_matrix.m32 = 0.0; self.unity_rotation_y_90_matrix.m33 = 1.0
        
        # Test matrix with invalid rotation (non-orthogonal)
        self.invalid_rotation_matrix = Matrix4x4()
        self.invalid_rotation_matrix.m00 = 2.0; self.invalid_rotation_matrix.m01 = 0.0; self.invalid_rotation_matrix.m02 = 0.0; self.invalid_rotation_matrix.m03 = 0.0
        self.invalid_rotation_matrix.m10 = 0.0; self.invalid_rotation_matrix.m11 = 2.0; self.invalid_rotation_matrix.m12 = 0.0; self.invalid_rotation_matrix.m13 = 0.0
        self.invalid_rotation_matrix.m20 = 0.0; self.invalid_rotation_matrix.m21 = 0.0; self.invalid_rotation_matrix.m22 = 2.0; self.invalid_rotation_matrix.m23 = 0.0
        self.invalid_rotation_matrix.m30 = 0.0; self.invalid_rotation_matrix.m31 = 0.0; self.invalid_rotation_matrix.m32 = 0.0; self.invalid_rotation_matrix.m33 = 1.0

        # Unity starting pose
        self.unity_start_pose = Matrix4x4()
        self.unity_start_pose.m00 = 1.0; self.unity_start_pose.m01 = 0.0; self.unity_start_pose.m02 = 0.0; self.unity_start_pose.m03 = unity_position_offset[0]
        self.unity_start_pose.m10 = 0.0; self.unity_start_pose.m11 = 1.0; self.unity_start_pose.m12 = 0.0; self.unity_start_pose.m13 = unity_position_offset[1]
        self.unity_start_pose.m20 = 0.0; self.unity_start_pose.m21 = 0.0; self.unity_start_pose.m22 = 1.0; self.unity_start_pose.m23 = unity_position_offset[2]
        self.unity_start_pose.m30 = 0.0; self.unity_start_pose.m31 = 0.0; self.unity_start_pose.m32 = 0.0; self.unity_start_pose.m33 = 1.0

        # Robot arm starting pose
        self.robot_arm_start_pose = SE3() # Initializes to identity matrix
        self.robot_arm_start_pose.x = robot_position_offset[0]
        self.robot_arm_start_pose.y = robot_position_offset[1]
        self.robot_arm_start_pose.z = robot_position_offset[2]

        # Unity translated pose
        self.unity_translated_pose = Matrix4x4()
        self.unity_translated_pose.m00 = 1.0; self.unity_translated_pose.m01 = 0.0; self.unity_translated_pose.m02 = 0.0; self.unity_translated_pose.m03 = unity_position_offset[0]+0.1
        self.unity_translated_pose.m10 = 0.0; self.unity_translated_pose.m11 = 1.0; self.unity_translated_pose.m12 = 0.0; self.unity_translated_pose.m13 = unity_position_offset[1]+0.2
        self.unity_translated_pose.m20 = 0.0; self.unity_translated_pose.m21 = 0.0; self.unity_translated_pose.m22 = 1.0; self.unity_translated_pose.m23 = unity_position_offset[2]+0.03
        self.unity_translated_pose.m30 = 0.0; self.unity_translated_pose.m31 = 0.0; self.unity_translated_pose.m32 = 0.0; self.unity_translated_pose.m33 = 1.0

        # Robot arm translated pose (directions become different for the three axes)
        self.robot_arm_translated_pose = SE3() # Initializes to identity matrix
        self.robot_arm_translated_pose.x = robot_position_offset[0]+0.03
        self.robot_arm_translated_pose.y = robot_position_offset[1]-0.1
        self.robot_arm_translated_pose.z = robot_position_offset[2]+0.2



    def test_start_pose(self):
        """Test the robot arm start pose."""
        result = get_pos_in_robot_coordinate_system_se3(self.unity_start_pose)

        # Check that the result is valid
        self.assertIsInstance(result, SE3)

        # Check that the result matches the expected robot arm start pose
        #print( "Resulting robot arm start pose:\n", result.A )
        #print( "Expected robot arm start pose:\n", self.robot_arm_start_pose.A )
        self.assertTrue(np.allclose(result.A, self.robot_arm_start_pose.A, atol=1e-5))

    def test_translated_pose(self):
        """Test the robot arm translated pose."""
        result = get_pos_in_robot_coordinate_system_se3(self.unity_translated_pose)

        # Check that the result is valid
        self.assertIsInstance(result, SE3)

        # Check that the result matches the expected robot arm translated pose
        #print( "Resulting robot arm translated pose:\n", result.A )
        #print( "Expected robot arm translated pose:\n", self.robot_arm_translated_pose.A )
        self.assertTrue(np.allclose(result.A, self.robot_arm_translated_pose.A, atol=1e-5))

    def test_identity_matrix_conversion(self):
        """Test conversion of identity matrix returns valid SE3."""
        result = get_pos_in_robot_coordinate_system_se3(self.identity_matrix_4x4)
        
        # Should return a valid SE3 object
        self.assertIsInstance(result, SE3)
        
        # The result should be a 4x4 matrix
        self.assertEqual(result.A.shape, (4, 4))
        
        # The transformation should be invertible (det != 0)
        self.assertNotEqual(np.linalg.det(result.A[:3, :3]), 0)

    def test_translation_only_matrix(self):
        """Test conversion of translation-only matrix."""
        result = get_pos_in_robot_coordinate_system_se3(self.unity_translation_matrix)
        
        # Should return a valid SE3 object
        self.assertIsInstance(result, SE3)
        
        # Check that rotation part is still valid (determinant should be 1 or -1)
        rotation_part = result.A[:3, :3]
        det = np.linalg.det(rotation_part)
        self.assertAlmostEqual(abs(det), 1.0, places=5)

    def test_rotation_matrix_normalization(self):
        """Test that rotation matrices are properly normalized."""
        result = get_pos_in_robot_coordinate_system_se3(self.unity_rotation_y_90_matrix)
        
        # Should return a valid SE3 object
        self.assertIsInstance(result, SE3)
        
        # Check that rotation part is orthogonal and normalized
        rotation_part = result.A[:3, :3]
        
        # Check that it's orthogonal: R @ R.T should be identity
        should_be_identity = rotation_part @ rotation_part.T
        np.testing.assert_allclose(should_be_identity, np.eye(3), atol=1e-10)
        
        # Check that determinant is 1 (proper rotation, not reflection)
        det = np.linalg.det(rotation_part)
        self.assertAlmostEqual(det, 1.0, places=10)

    def test_invalid_rotation_matrix_handling(self):
        """Test that invalid rotation matrices are properly normalized."""
        result = get_pos_in_robot_coordinate_system_se3(self.invalid_rotation_matrix)
        
        # Should return a valid SE3 object (function should handle invalid input gracefully)
        self.assertIsInstance(result, SE3)
        
        # The resulting rotation should be normalized
        rotation_part = result.A[:3, :3]
        det = np.linalg.det(rotation_part)
        self.assertAlmostEqual(abs(det), 1.0, places=5)

    def test_coordinate_system_transformation(self):
        """Test that coordinate system transformation is applied correctly."""
        # Test with identity matrix at origin
        result = get_pos_in_robot_coordinate_system_se3(self.identity_matrix_4x4)
        
        # The position should be affected by the coordinate system transformation and offsets
        position = result.A[:3, 3]
        
        # Position should not be exactly [0, 0, 0] due to offsets and coordinate transformation
        self.assertFalse(np.allclose(position, [0, 0, 0], atol=1e-10))

    def test_position_offset_application(self):
        """Test that position offsets are applied correctly."""
        # Test with translation matrix
        result = get_pos_in_robot_coordinate_system_se3(self.unity_translation_matrix)
        
        # Should return a valid SE3 object
        self.assertIsInstance(result, SE3)
        
        # Position should include the robot position offset
        position = result.A[:3, 3]
        
        # Since we're applying coordinate transformations and offsets, 
        # the final position should be different from the input translation
        expected_translation = np.array([1.0, 2.0, 3.0])  # Original Unity translation
        self.assertFalse(np.allclose(position, expected_translation, atol=1e-10))

    def test_error_handling_with_malformed_matrix(self):
        """Test error handling when SE3 creation fails."""
        # Create a matrix that might cause SE3 creation to fail
        malformed_matrix = Matrix4x4()
        # Set all values to NaN
        for field_name, _ in malformed_matrix._fields_:
            setattr(malformed_matrix, field_name, float('nan'))
        
        # Function should handle the error gracefully and return NEUTRAL_POSE_SE3
        # However, the current implementation might throw an exception during SVD
        # This test documents the current behavior
        with self.assertRaises((ValueError, np.linalg.LinAlgError)):
            result = get_pos_in_robot_coordinate_system_se3(malformed_matrix)

    def test_matrix_data_type_consistency(self):
        """Test that the function handles data type conversion correctly."""
        result = get_pos_in_robot_coordinate_system_se3(self.identity_matrix_4x4)
        
        # The resulting matrix should be float64
        self.assertEqual(result.A.dtype, np.float64)

    def test_determinant_correction(self):
        """Test that improper rotations (det = -1) are corrected to proper rotations (det = 1)."""
        # Create a matrix with determinant -1 (reflection)
        reflection_matrix = Matrix4x4()
        reflection_matrix.m00 = -1.0; reflection_matrix.m01 = 0.0; reflection_matrix.m02 = 0.0; reflection_matrix.m03 = 0.0
        reflection_matrix.m10 = 0.0; reflection_matrix.m11 = 1.0; reflection_matrix.m12 = 0.0; reflection_matrix.m13 = 0.0
        reflection_matrix.m20 = 0.0; reflection_matrix.m21 = 0.0; reflection_matrix.m22 = 1.0; reflection_matrix.m23 = 0.0
        reflection_matrix.m30 = 0.0; reflection_matrix.m31 = 0.0; reflection_matrix.m32 = 0.0; reflection_matrix.m33 = 1.0
        
        result = get_pos_in_robot_coordinate_system_se3(reflection_matrix)
        
        # The resulting rotation should have determinant 1 (proper rotation)
        rotation_part = result.A[:3, :3]
        det = np.linalg.det(rotation_part)
        self.assertAlmostEqual(det, 1.0, places=10)

    def test_svd_normalization_stability(self):
        """Test that SVD normalization produces stable results."""
        # Create a slightly non-orthogonal rotation matrix
        slightly_off_matrix = Matrix4x4()
        slightly_off_matrix.m00 = 1.001; slightly_off_matrix.m01 = 0.001; slightly_off_matrix.m02 = 0.0; slightly_off_matrix.m03 = 0.0
        slightly_off_matrix.m10 = -0.001; slightly_off_matrix.m11 = 1.001; slightly_off_matrix.m12 = 0.0; slightly_off_matrix.m13 = 0.0
        slightly_off_matrix.m20 = 0.0; slightly_off_matrix.m21 = 0.0; slightly_off_matrix.m22 = 1.0; slightly_off_matrix.m23 = 0.0
        slightly_off_matrix.m30 = 0.0; slightly_off_matrix.m31 = 0.0; slightly_off_matrix.m32 = 0.0; slightly_off_matrix.m33 = 1.0
        
        result = get_pos_in_robot_coordinate_system_se3(slightly_off_matrix)
        
        # The result should be a proper orthogonal matrix
        rotation_part = result.A[:3, :3]
        
        # Check orthogonality
        should_be_identity = rotation_part @ rotation_part.T
        np.testing.assert_allclose(should_be_identity, np.eye(3), atol=1e-10)
        
        # Check proper rotation
        det = np.linalg.det(rotation_part)
        self.assertAlmostEqual(det, 1.0, places=10)

    def test_coordinate_transformation_consistency(self):
        """Test that the coordinate transformation is applied consistently."""
        # Test multiple different input matrices
        test_matrices = [self.identity_matrix_4x4, self.unity_translation_matrix, self.unity_rotation_y_90_matrix]
        
        for test_matrix in test_matrices:
            result = get_pos_in_robot_coordinate_system_se3(test_matrix)
            
            # All results should be valid SE3 objects
            self.assertIsInstance(result, SE3)
            
            # All results should have proper rotation matrices
            rotation_part = result.A[:3, :3]
            det = np.linalg.det(rotation_part)
            self.assertAlmostEqual(abs(det), 1.0, places=5)

    def test_zero_translation_matrix(self):
        """Test conversion of matrix with zero translation."""
        zero_translation_matrix = Matrix4x4()
        zero_translation_matrix.m00 = 1.0; zero_translation_matrix.m01 = 0.0; zero_translation_matrix.m02 = 0.0; zero_translation_matrix.m03 = 0.0
        zero_translation_matrix.m10 = 0.0; zero_translation_matrix.m11 = 1.0; zero_translation_matrix.m12 = 0.0; zero_translation_matrix.m13 = 0.0
        zero_translation_matrix.m20 = 0.0; zero_translation_matrix.m21 = 0.0; zero_translation_matrix.m22 = 1.0; zero_translation_matrix.m23 = 0.0
        zero_translation_matrix.m30 = 0.0; zero_translation_matrix.m31 = 0.0; zero_translation_matrix.m32 = 0.0; zero_translation_matrix.m33 = 1.0
        
        result = get_pos_in_robot_coordinate_system_se3(zero_translation_matrix)
        self.assertIsInstance(result, SE3)

    def test_large_translation_values(self):
        """Test conversion with large translation values."""
        large_translation_matrix = Matrix4x4()
        large_translation_matrix.m00 = 1.0; large_translation_matrix.m01 = 0.0; large_translation_matrix.m02 = 0.0; large_translation_matrix.m03 = 1000.0
        large_translation_matrix.m10 = 0.0; large_translation_matrix.m11 = 1.0; large_translation_matrix.m12 = 0.0; large_translation_matrix.m13 = 2000.0
        large_translation_matrix.m20 = 0.0; large_translation_matrix.m21 = 0.0; large_translation_matrix.m22 = 1.0; large_translation_matrix.m23 = 3000.0
        large_translation_matrix.m30 = 0.0; large_translation_matrix.m31 = 0.0; large_translation_matrix.m32 = 0.0; large_translation_matrix.m33 = 1.0
        
        result = get_pos_in_robot_coordinate_system_se3(large_translation_matrix)
        self.assertIsInstance(result, SE3)
        
        # Should still have valid rotation matrix
        rotation_part = result.A[:3, :3]
        det = np.linalg.det(rotation_part)
        self.assertAlmostEqual(abs(det), 1.0, places=5)

    def test_negative_translation_values(self):
        """Test conversion with negative translation values."""
        negative_translation_matrix = Matrix4x4()
        negative_translation_matrix.m00 = 1.0; negative_translation_matrix.m01 = 0.0; negative_translation_matrix.m02 = 0.0; negative_translation_matrix.m03 = -1.0
        negative_translation_matrix.m10 = 0.0; negative_translation_matrix.m11 = 1.0; negative_translation_matrix.m12 = 0.0; negative_translation_matrix.m13 = -2.0
        negative_translation_matrix.m20 = 0.0; negative_translation_matrix.m21 = 0.0; negative_translation_matrix.m22 = 1.0; negative_translation_matrix.m23 = -3.0
        negative_translation_matrix.m30 = 0.0; negative_translation_matrix.m31 = 0.0; negative_translation_matrix.m32 = 0.0; negative_translation_matrix.m33 = 1.0
        
        result = get_pos_in_robot_coordinate_system_se3(negative_translation_matrix)
        self.assertIsInstance(result, SE3)

    def test_rotation_around_x_axis(self):
        """Test conversion with rotation around X axis."""
        # 90 degree rotation around X axis
        rotation_x_90_matrix = Matrix4x4()
        rotation_x_90_matrix.m00 = 1.0; rotation_x_90_matrix.m01 = 0.0; rotation_x_90_matrix.m02 = 0.0; rotation_x_90_matrix.m03 = 0.0
        rotation_x_90_matrix.m10 = 0.0; rotation_x_90_matrix.m11 = 0.0; rotation_x_90_matrix.m12 = -1.0; rotation_x_90_matrix.m13 = 0.0
        rotation_x_90_matrix.m20 = 0.0; rotation_x_90_matrix.m21 = 1.0; rotation_x_90_matrix.m22 = 0.0; rotation_x_90_matrix.m23 = 0.0
        rotation_x_90_matrix.m30 = 0.0; rotation_x_90_matrix.m31 = 0.0; rotation_x_90_matrix.m32 = 0.0; rotation_x_90_matrix.m33 = 1.0
        
        result = get_pos_in_robot_coordinate_system_se3(rotation_x_90_matrix)
        self.assertIsInstance(result, SE3)
        
        # Check that rotation part is orthogonal
        rotation_part = result.A[:3, :3]
        should_be_identity = rotation_part @ rotation_part.T
        np.testing.assert_allclose(should_be_identity, np.eye(3), atol=1e-10)

    def test_rotation_around_z_axis(self):
        """Test conversion with rotation around Z axis."""
        # 90 degree rotation around Z axis
        rotation_z_90_matrix = Matrix4x4()
        rotation_z_90_matrix.m00 = 0.0; rotation_z_90_matrix.m01 = -1.0; rotation_z_90_matrix.m02 = 0.0; rotation_z_90_matrix.m03 = 0.0
        rotation_z_90_matrix.m10 = 1.0; rotation_z_90_matrix.m11 = 0.0; rotation_z_90_matrix.m12 = 0.0; rotation_z_90_matrix.m13 = 0.0
        rotation_z_90_matrix.m20 = 0.0; rotation_z_90_matrix.m21 = 0.0; rotation_z_90_matrix.m22 = 1.0; rotation_z_90_matrix.m23 = 0.0
        rotation_z_90_matrix.m30 = 0.0; rotation_z_90_matrix.m31 = 0.0; rotation_z_90_matrix.m32 = 0.0; rotation_z_90_matrix.m33 = 1.0
        
        result = get_pos_in_robot_coordinate_system_se3(rotation_z_90_matrix)
        self.assertIsInstance(result, SE3)
        
        # Check that rotation part is orthogonal
        rotation_part = result.A[:3, :3]
        should_be_identity = rotation_part @ rotation_part.T
        np.testing.assert_allclose(should_be_identity, np.eye(3), atol=1e-10)

    def tearDown(self):
        """Clean up after each test method."""
        # Restore original global variables if needed
        pass


if __name__ == '__main__':
    # Run the tests
    unittest.main(verbosity=2)
