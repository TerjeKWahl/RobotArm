using UnityEngine;
using System;

public class RobotMessageManager
{
    /// <summary>
    /// Creates a message byte array from VR to PC following the specified format:
    /// - Prefix "TKW"
    /// - API version (1)
    /// - Information source (20 for VR app)
    /// - Reserved bytes (3 bytes set to 0)
    /// - 16 double floating point numbers representing the 4x4 matrix (128 bytes)
    /// Total message size: 136 bytes
    /// </summary>
    /// <param name="matrix">The 4x4 transformation matrix from the VR gripper</param>
    /// <returns>Byte array containing the formatted message</returns>
    public byte[] CreateMessageFromVRToPC(Matrix4x4 matrix)
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

        // 6-8 are reserved for future use, set to 0
        message[offset++] = 0;
        message[offset++] = 0;
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
