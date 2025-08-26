using UnityEngine;
using System;

public class RobotMessageManager
{
    /// <summary>
    /// Creates a message byte array from VR to PC following the format
    /// defined in Docs/Api.md.
    /// </summary>
    /// <param name="matrix">The 4x4 transformation matrix from the VR gripper</param>
    /// <param name="desiredGripperAngle">The desired gripper angle</param>
    /// <param name="isRecording">Whether the recording is active</param>
    /// <returns>Byte array containing the formatted message</returns>
    public byte[] CreateMessageFromVRToPC(Matrix4x4 matrix, byte desiredGripperAngle, bool isRecording)
    {
        const int MESSAGE_SIZE = 136;
        byte[] message = new byte[MESSAGE_SIZE];
        int offset = 0;

        // 1. Prefix "T"
        message[offset++] = (byte)'T';
        
        // 2. Prefix "K"
        message[offset++] = (byte)'K';
        
        // 3. Prefix "W"
        message[offset++] = (byte)'W';
        
        // 4. API version, 1
        message[offset++] = 1;
        
        // 5. Information source: 20 = VR app
        message[offset++] = 20;

        // 6. Desired gripper angle
        message[offset++] = desiredGripperAngle;

        // 7. Recording status
        message[offset++] = (byte)(isRecording ? 1 : 0);

        // 8. Reserved for future use, set to 0
        message[offset++] = 0;

        // 9. Bytes 9 to 136: 16 double floating point numbers representing the 4x4 matrix
        // Matrix data is stored row-first
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                double value = (double)matrix[row, col];
                byte[] doubleBytes = BitConverter.GetBytes(value);
                
                // Ensure little-endian byte order for consistency
                if (!BitConverter.IsLittleEndian)
                {
                    Array.Reverse(doubleBytes);
                }
                
                Array.Copy(doubleBytes, 0, message, offset, 8);
                offset += 8;
            }
        }
        
        return message;
    }
}
