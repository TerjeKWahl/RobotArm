using UnityEngine;
using TMPro;

public class UITextUpdater : MonoBehaviour
{
    public GameObject RobotGripper;
    private TMP_Text textMeshPro;
    private readonly UdpManager udpManager = new UdpManager(Configuration.PC_IP_ADDRESS, Configuration.PC_PORT_NUMBER);
    private RobotMessageManager robotMessageManager = new RobotMessageManager();
    private Matrix4x4 startPositionMatrix = Matrix4x4.zero;
    private Vector3 startPosition = Vector3.zero;
    private Vector3 startRotation = Vector3.zero;
    private bool isRecording = false; // Used to track status of recording on/off, for signaling to PC when to start/stop recording training videoes
    private bool isAPressedLast = false; // To track change in A button state

    void Start()
    {
        textMeshPro = GetComponent<TMP_Text>();
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
            // Initialize start position and rotation matrices if they are not set yet
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

            // Get the current position and rotation of the RobotGripper
            Matrix4x4 positionMatrix = RobotGripper.transform.localToWorldMatrix;

            Vector3 position = RobotGripper.transform.position;
            Vector3 rotation = RobotGripper.transform.eulerAngles;
            // Convert to mm, and note the x,y,z axis changes because of different coordinate system for robot arm:
            float posX = (position.z - startPosition.z) * 1000;
            float posY = (position.x - startPosition.x) * -1000;
            float posZ = (position.y - startPosition.y) * 1000;
            float rotX = rotation.x - startRotation.x;
            float rotY = rotation.z - startRotation.z;
            float rotZ = rotation.y - startRotation.y;
            if (rotX > 180) rotX -= 360; // Want to keep the range of rotation values between -180 and 180 degrees
            if (rotY > 180) rotY -= 360;
            if (rotZ > 180) rotZ -= 360;

            // Get current VR controller right index trigger button value:
            float rightTriggerValue = OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger);

            bool isAPressed = OVRInput.Get(OVRInput.Button.One); // True if A button on right controller is currently pressed
            if (isAPressed && !isAPressedLast)
            {
                isRecording = !isRecording;
                Debug.Log($"Recording status changed: {isRecording}");
            }
            isAPressedLast = isAPressed;


            int gripperAngle = 45 - (int)(rightTriggerValue * 45);

            // Format position and rotation strings
            string posXStr = FormatNumber(posX);
            string posYStr = FormatNumber(posY);
            string posZStr = FormatNumber(posZ);
            string rotXStr = FormatNumber(rotX);
            string rotYStr = FormatNumber(rotY);
            string rotZStr = FormatNumber(rotZ);

            string text = $"Position (mm):  (X, Y, Z): {posXStr} {posYStr} {posZStr}\n" +
                           $"Orientation (°)   (X, Y, Z): {rotXStr} {rotYStr} {rotZStr}\n" +
                           $"(X is forward, Y is to the left, Z is up)\n" +
                           $"Gripper angle (0 is closed): {gripperAngle}°\n" +
                           $"Recording: {(isRecording ? "On" : "Off")}\n" +
                           $"Current pos matrix: \n{positionMatrix}\n"
                           //$"Gripper (0-100% pressed): {(int)(rightTriggerValue * 100)}%\n" +
                           //$"A button pressed: {isAPressed}\n"
                           ;
            textMeshPro.text = text;

            // Send UDP packet, but only every FRAMES_SEND_MESSAGE_PERIOD frames to avoid flooding the network
            if (Time.frameCount % Configuration.FRAMES_SEND_MESSAGE_PERIOD == 0)
            {
                byte[] messageBytes = robotMessageManager.CreateMessageFromVRToPC(positionMatrix);
                udpManager.SendMessage(messageBytes);
            }
        }
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

}
