public class Configuration
{
    public const int FRAMES_SEND_MESSAGE_PERIOD = 2; // How often to send UDP messages to the PC, related 
                                                     // to framerate (and execution of update() function)
    public const string PC_IP_ADDRESS = "192.168.0.43";
    public const int PC_PORT_NUMBER = 7506;
    public const byte GRIPPER_ANGLE_MAX_DEG = 45;
}
