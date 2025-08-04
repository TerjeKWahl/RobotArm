using UnityEngine;
using TMPro;

public class UITextUpdater : MonoBehaviour
{
    public GameObject RobotGripper;
    private TMP_Text textMeshPro;
    private Matrix4x4 startPositionMatrix = Matrix4x4.zero;
    // Definition of transformation matrix for conversion from 
    // Unity's coordinate system (x-right, y-up, z-forward)
    // to the robot arm's coordinate system (x-forward, y-left, z-up):
    /*private Matrix4x4 translationAndRotationMatrix = new Matrix4x4(
        new Vector4(0, -1, 0, 0),  // Column 0
        new Vector4(0, 0, 1, 0),   // Column 1
        new Vector4(1, 0, 0, 0),   // Column 2
        new Vector4(0, 0, 0, 1)    // Column 3, including translation which is 0 here because 
                                   // it will be saved here later on startup
    );*/
    private Vector3 startPosition = Vector3.zero;
    private Vector3 startRotation = Vector3.zero;

    void Start()
    {
        textMeshPro = GetComponent<TMP_Text>();
    }

    /// <summary>
    /// Create text string with whole numbers from a float. 
    /// If the value is negative, add special wide minus sign "–". 
    /// If value is 0 or positive, add two spaces before the number.
    /// If absolute value of number is less than 10, add four spaces before the number.
    /// If absolute value of number is less than 100, add two spaces before the number.
    /// </summary>
    private string FormatNumber(float value)
    {
        int roundedValue = Mathf.RoundToInt(value);
        string sign = roundedValue < 0 ? "–" : "  ";
        int absValue = Mathf.Abs(roundedValue);

        if (absValue < 10)
            return $"    {sign}{absValue}";
        else if (absValue < 100)
            return $"  {sign}{absValue}";
        else
            return $"{sign}{absValue}";
    }

    /// <summary>
    /// Update the text with the current position and rotation of the RobotGripper,
    /// as an offset from the starting position and rotation, in the coordinate system of the
    /// robot arm (not Unity's coordinate system).
    /// </summary>
    void Update()
    {
        if (RobotGripper != null)
        {
            if (startPositionMatrix == Matrix4x4.zero)
            {
                startPositionMatrix = RobotGripper.transform.localToWorldMatrix;
                Debug.Log($"Start position matrix initialized to \n{startPositionMatrix}.");
                //startPosition = RobotGripper.transform.position;
                //translationAndRotationMatrix.SetColumn(3,
                //    new Vector4(-startPosition.x, -startPosition.y, -startPosition.z, 1));
                //Debug.Log($"Translation and rotation matrix updated to \n{translationAndRotationMatrix}.");
            }
            if (startPosition == Vector3.zero)
            {
                startPosition = RobotGripper.transform.position;
            }
            if (startRotation == Vector3.zero)
            {
                startRotation = RobotGripper.transform.eulerAngles;
            }

            Matrix4x4 positionMatrix = RobotGripper.transform.localToWorldMatrix;
            //Matrix4x4 offsetMatrix = positionMatrix * startPositionMatrix.inverse;
            //Matrix4x4 convertedMatrix = positionMatrix * translationAndRotationMatrix;

            Vector3 position = RobotGripper.transform.position;
            Vector3 rotation = RobotGripper.transform.eulerAngles;
            // Convert to mm, and note the x,y,z axis changes because of different coordinate system for robot arm:
            float posX = (position.z - startPosition.z) * 1000; 
            float posY = (position.x - startPosition.x) * -1000;
            float posZ = (position.y - startPosition.y) * 1000;
            float rotX = rotation.x - startRotation.x;
            float rotY = rotation.z - startRotation.z;
            float rotZ = rotation.y - startRotation.y;
            if(rotX > 180) rotX -= 360; // Want to keep the range of rotation values between -180 and 180 degrees
            if(rotY > 180) rotY -= 360;
            if(rotZ > 180) rotZ -= 360;

            // Format position and rotation strings
            string posXStr = FormatNumber(posX);
            string posYStr = FormatNumber(posY);
            string posZStr = FormatNumber(posZ);
            string rotXStr = FormatNumber(rotX);
            string rotYStr = FormatNumber(rotY);
            string rotZStr = FormatNumber(rotZ);

            string text = $"Position (mm):  (X, Y, Z): {posXStr} {posYStr} {posZStr}\n" +
                           $"Orientation (°)   (X, Y, Z): {rotXStr} {rotYStr} {rotZStr}\n" +
                           $"X is forward\n" +
                           $"Y is to the left\n" +
                           $"Z is up\n" +
                           //$"Start pos matrix: \n{startPositionMatrix}.\n" +
                           //$"translationAndRotationMatrix: \n{translationAndRotationMatrix}.\n" +
                           $"Current pos matrix: \n{positionMatrix}.\n" +
                           //$"Offset matrix: \n{offsetMatrix}.\n" +
                           //$"Converted matrix: \n{convertedMatrix}.\n" +
                           //$"RobotGripper.transform.localScale: {RobotGripper.transform.localScale}.\n"
                           //$"RobotGripper.transform.localToWorldMatrix: \n{RobotGripper.transform.localToWorldMatrix}.\n" +
                           //$"Connection status:\n" +
                           //$"PC: 	Not connected\n" + // TODO: Implement actual connection status
                           //$"Controllers:	Not connected\n" +
                           ;
            textMeshPro.text = text;
        }
    }
}
