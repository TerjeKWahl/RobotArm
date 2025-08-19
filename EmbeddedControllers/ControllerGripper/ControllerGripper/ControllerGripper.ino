/**
 * Controlling a servo using an Arduino Uno R4 WiFi. 
 * 
 * Servo used: 
 * Geekservo 2KG Mini Servo 360 Degree
 * ------------
 * Weight: 20g
 * Operating voltage: 4.8V - 6.0V
 * Operating speed (at no load) 0.14±0.01sec/60° 0.12±0.01sec/60°
 * Running current (at no load) 70±20mA 90±20mA
 * Stall torque (at locked) 1.8±0.2kg-cm 2±0.2kg-cm
 * Stall current (at locked) 0.8±0.1A 0.9±0.1A
 * Neutral position:1500μsec
 * Dead band width: ≤4μsec
 * Rotating direction: counter Clockwise (1500→500μsec)
 * Pulse width range: 500→2500 μsec
 * Maximum travel 360°(500→2500 μsec)
 *
 * Wiring:
 * Brown: GND
 * Red: VCC (4.8 ~ 6.6v)
 * Orange: PWM Signal
 */

#include "Arduino_LED_Matrix.h"
#include <Servo.h>
#include <WiFiS3.h>
#include "WifiSecrets.h"    // Please enter your sensitive data into WifiSecrets.h, by defining SECRET_SSID and SECRET_PASS there.
#include "Configuration.h"
#include "RobotMessageManager.h"

const int16_t SERVO_PIN = 9;
const uint32_t MAX_WAIT_TIME_FOR_SERIAL_MS = 3000; // Maximum wait time for serial connection to start working on startup, in milliseconds

Servo gripperServo; 

int wifiStatus = WL_IDLE_STATUS;
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
char receiveBuffer[256]; //buffer to hold incoming packet
char sendBuffer[256] = "TKW"; // a string to send back
WiFiUDP udp;

ArduinoLEDMatrix matrix;
uint32_t frame[] = { 0x00000000, 0x00000000, 0x00000000 }; // 1-bit display buffer for the 12 x 8 pixel display


void setup() 
{
    Serial.begin(115200);

    matrix.begin();
    matrix.loadFrame(LEDMATRIX_CHIP); // Boot screen
    
    gripperServo.attach(SERVO_PIN);
    gripperServo.writeMicroseconds(getServoUsFromDegrees(GRIPPER_ANGLE_MAX_DEG));

    while (!Serial && (millis() < MAX_WAIT_TIME_FOR_SERIAL_MS)) {
        ; // Wait for serial port to connect.
    }
    while (millis() < MAX_WAIT_TIME_FOR_SERIAL_MS) 
    {
        delay(10); // Give some time for the serial connection to stabilize, if it was plugged in.
    }

    Serial.println("\nControllerGripper started!");

    matrix.loadFrame(LEDMATRIX_CLOUD_WIFI);
    connectToWifi();

    Serial.println("\nStarting connection to server...");
    udp.begin(ARDUINO_UDP_PORT);

    Serial.println("Setup complete, ready to control the gripper.");
    matrix.loadFrame(LEDMATRIX_HEART_BIG);
    delay(500);
    matrix.loadFrame(LEDMATRIX_EMOJI_HAPPY);
    delay(500);
}

  
void loop() 
{

    while(true)
    {
        // Demo mode
        uint32_t demoDurationWantedMs = 20000; // Duration of demo mode before going into normal remote control function. Set to 0 to skip demo mode.
        uint32_t currentTimeMs = millis();
        uint32_t demoStartMs = currentTimeMs;
        int16_t servoPositionDegrees = 0;
        if (demoDurationWantedMs > 0)
        {
            Serial.println("Starting demo mode");
            while (currentTimeMs - demoStartMs < demoDurationWantedMs) {
                uint32_t timeInPeriod = (currentTimeMs - demoStartMs) % 10000; // 10 seconds cycle

                if (timeInPeriod < 2000) 
                {
                    servoPositionDegrees = map(timeInPeriod, 0, 2000, GRIPPER_ANGLE_MIN_DEG, GRIPPER_ANGLE_MAX_DEG);
                } 
                else if (timeInPeriod < 4000) 
                {
                    servoPositionDegrees = map(timeInPeriod, 2000, 4000, GRIPPER_ANGLE_MAX_DEG, GRIPPER_ANGLE_MIN_DEG);
                } 
                else if (timeInPeriod < 5000) 
                {
                    servoPositionDegrees = GRIPPER_ANGLE_MIN_DEG;
                } 
                else if (timeInPeriod < 6000) 
                {
                    servoPositionDegrees = map(timeInPeriod, 5000, 6000, GRIPPER_ANGLE_MIN_DEG, GRIPPER_ANGLE_MAX_DEG);
                } 
                else if (timeInPeriod < 7000) 
                {
                    servoPositionDegrees = map(timeInPeriod, 6000, 7000, GRIPPER_ANGLE_MAX_DEG, GRIPPER_ANGLE_MIN_DEG);
                } 
                else if (timeInPeriod < 7500) 
                {
                    servoPositionDegrees = map(timeInPeriod, 7000, 7500, GRIPPER_ANGLE_MIN_DEG, GRIPPER_ANGLE_MAX_DEG);
                } 
                else if (timeInPeriod < 8000) 
                {
                    servoPositionDegrees = map(timeInPeriod, 7500, 8000, GRIPPER_ANGLE_MAX_DEG, GRIPPER_ANGLE_MIN_DEG);
                } 
                else 
                {
                    servoPositionDegrees = GRIPPER_ANGLE_MIN_DEG;
                }

                moveGripper(servoPositionDegrees);

                currentTimeMs = millis(); // Update current time for next iteration of the while loop
            }
            Serial.println("Demo mode finished, entering normal operation mode.");
        }


        // Normal operation
        while(true) // Outer loop to handle errors and reconnects
        {
            int16_t servoPositionDegrees = GRIPPER_ANGLE_MAX_DEG; // Start with fully opened gripper
            moveGripper(servoPositionDegrees);
            int32_t lastSendTimeMs = 0;

            while(true) { // Inner loop to handle incoming UDP messages and send updates
                
                // Check if WiFi is still connected (WiFi.status() == WL_CONNECTED) at regular intervals.
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("WiFi disconnected!");
                    break; // Exit the inner loop to reconnect
                }

                // Update the gripper position based on any incoming UDP messages
                int packetSize = udp.parsePacket();
                if (packetSize) {
                    Serial.print("Received packet of size ");
                    Serial.print(packetSize);
                    Serial.print(" from ");
                    IPAddress remoteIp = udp.remoteIP();
                    Serial.print(remoteIp);
                    Serial.print(", port ");
                    Serial.println(udp.remotePort());

                    uint8_t receiveBuffer[REC_MSG_LENGTH];
                    if (packetSize > REC_MSG_LENGTH) {
                        Serial.println("Packet size exceeds buffer size. Ignoring it.");
                        udp.flush();
                    }
                    else
                    {
                        int len = udp.read(receiveBuffer, REC_MSG_LENGTH);
                        if (len == 0) {
                            Serial.println("Failed to read UDP packet.");
                            break; // Exit the inner loop to reconnect
                        }
                        // Parse the incoming message
                        bool isSuccessful = false;
                        MessageFromPcToController incomingMessage = parseMessageFromPcToController(receiveBuffer, len, &isSuccessful);
                        if (!isSuccessful) {
                            Serial.println("Failed to parse incoming message.");
                            break; // Exit the inner loop to reconnect
                        }
                        int16_t servoPositionDegrees = incomingMessage.desiredAngles.gripper;
                        Serial.print("New desired gripper angle: ");
                        Serial.println(servoPositionDegrees);
                        moveGripper(servoPositionDegrees);
                    }
                }


                // Send UDP message to the PC every SEND_PERIOD_MS milliseconds
                uint32_t currentTimeMs = millis();
                if (currentTimeMs - lastSendTimeMs >= SEND_PERIOD_MS) {
                    lastSendTimeMs = currentTimeMs;
                    udp.beginPacket(PC_IP_ADDRESS, ARDUINO_UDP_PORT);
                    uint8_t sendBuffer[SEND_MSG_LENGTH];
                    createMessageFromControllerToPc(servoPositionDegrees, sendBuffer);
                    udp.write(sendBuffer, SEND_MSG_LENGTH);
                    int16_t isSentSuccessfully = udp.endPacket();
                    if (!isSentSuccessfully) {
                        Serial.println("Failed to send UDP packet!");
                        break; // Exit the inner loop to reconnect
                    }
                }
            }

            // Disconnect and reconnect to WiFi
            Serial.println("Disconnecting from WiFi, and attempting reconnect...");
            udp.flush();
            udp.stop();
            WiFi.end();
            connectToWifi();
        }


    }
}


/**
 * Moves the gripper servo to the specified position and updates the LED matrix display.
 * 
 * @param targetDegrees: Target position for the gripper servo in degrees
 */
void moveGripper(int16_t targetDegrees) {
    gripperServo.writeMicroseconds(getServoUsFromDegrees(targetDegrees));
    updateFrame(frame, targetDegrees);
    matrix.loadFrame(frame);
}


/**
 * Updates the frame that represents the 1-bit display buffer for the 12 x 8 pixel display.
 * Lights up from 0 to 12 pixels on the first two rows, depending on the servoPositionDegrees that
 * is from GRIPPER_ANGLE_MIN_DEG to GRIPPER_ANGLE_MAX_DEG degrees.
 */
void updateFrame(uint32_t *frame, int servoPositionDegrees) 
{
    // Check for valid servo position by clipping it to the valid range
    if (servoPositionDegrees < GRIPPER_ANGLE_MIN_DEG) 
    {
        servoPositionDegrees = GRIPPER_ANGLE_MIN_DEG;
    } 
    else if (servoPositionDegrees > GRIPPER_ANGLE_MAX_DEG) 
    {
        servoPositionDegrees = GRIPPER_ANGLE_MAX_DEG;
    }
    // Could do some error checking here on the frame buffer, but hey this is a hobby project.

    // Clear the frame first (reset all bits except type "TW" in the middle of the display))
    frame[0] = 0b00000000000000000000000001110100;
    frame[1] = 0b01000010010101000010001010000010;
    frame[2] = 0b00101000000000000000000000000000;
    
    // Map servo position to number of pixels (0-12)
    int numPixels = map(servoPositionDegrees, GRIPPER_ANGLE_MIN_DEG, GRIPPER_ANGLE_MAX_DEG, 0, 12);

    // Set pixels in first two rows
    for (int bitPosition = 0; bitPosition < numPixels; bitPosition++) {
        // Set pixel in first row (bits 31-20)
        frame[0] |= (1UL << (31 - bitPosition));
        // Set pixel in second row (bits 19-8)
        frame[0] |= (1UL << (19 - bitPosition));
    }
}


void connectToWifi() 
{
    Serial.println("Connecting to WiFi network...");

    // Check for the presence of the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        // Don't continue
        while (true);
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("(Please consider upgrading the WiFi module firmware.)");
    }

    // Attempt to connect to WiFi network:
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifiStatus = WiFi.begin(ssid, pass);
    // wait up to 10 seconds for connection:
    int secondsWaited = 0;
    while (wifiStatus != WL_CONNECTED && secondsWaited < 20) 
    {
        Serial.println("Waiting for connection...");
        delay(1000);
        wifiStatus = WiFi.status();
        secondsWaited++;
    }
    IPAddress ip = WiFi.localIP();
    while ((uint32_t)ip == 0 && secondsWaited < 20) 
    {
        Serial.println("Waiting for IP address...");
        delay(1000);
        ip = WiFi.localIP();
        secondsWaited++;
    }
    Serial.println("Connected to WiFi!");
    printWifiStatus();
}


void printWifiStatus() {
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}


/**
 * Converts servo position in degrees to microseconds for PWM control.
 * Standard servo control uses pulse widths between 500-2500 microseconds.
 * 
 * This function is customized to fit with the servo and physical robot arm being used,
 * to match the minimum and maximum angles to the physical robot arm.
 * 
 * @param degrees: Servo position in degrees
 * @return: Pulse width in microseconds
 */
uint16_t getServoUsFromDegrees(int16_t degrees) {
    // Constrain degrees to valid servo range (0-180)
    if (degrees < GRIPPER_ANGLE_MIN_DEG) {
        degrees = GRIPPER_ANGLE_MIN_DEG;
    } else if (degrees > GRIPPER_ANGLE_MAX_DEG) {
        degrees = GRIPPER_ANGLE_MAX_DEG;
    }

    // Map degrees (GRIPPER_ANGLE_MIN_DEG-GRIPPER_ANGLE_MAX_DEG) to microseconds
    const int16_t US_MIN = 1500 + 25; // Don't squeeze too tightly
    const int16_t US_MAX = 1750; // 1500 + 5.55 per degree. So for 45 degrees it is 1500+250 = 1750
    const int16_t SPAN = GRIPPER_ANGLE_MAX_DEG - GRIPPER_ANGLE_MIN_DEG;
    return US_MIN + ((uint32_t)degrees * (US_MAX - US_MIN)) / SPAN;
}


