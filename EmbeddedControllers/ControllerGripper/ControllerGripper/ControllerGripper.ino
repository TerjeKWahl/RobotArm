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

const int16_t SERVO_PIN = 9;
const uint32_t MAX_WAIT_TIME_FOR_SERIAL_MS = 3000; // Maximum wait time for serial connection to start working on startup, in milliseconds

Servo gripperServo; 
int16_t servoPositionDegrees = 0;

int wifiStatus = WL_IDLE_STATUS;
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
// Also, make a DHCP reservation on your router for this device, so that it always gets the same IP address that can be used to control the gripper.
int keyIndex = 0;           // your network key index number (needed only for WEP)
unsigned int localNetworkPort = 2390; // local port to listen on
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
        uint32_t milliseconds = millis();
        uint32_t timeInPeriod = milliseconds % 10000; // 10 seconds cycle

        if (timeInPeriod < 2000) 
        {
            servoPositionDegrees = map(timeInPeriod, 0, 2000, 0, 180);
        } 
        else if (timeInPeriod < 4000) 
        {
            servoPositionDegrees = map(timeInPeriod, 2000, 4000, 180, 0);
        } 
        else if (timeInPeriod < 5000) 
        {
            servoPositionDegrees = 0;
        } 
        else if (timeInPeriod < 6000) 
        {
            servoPositionDegrees = map(timeInPeriod, 5000, 6000, 0, 180);
        } 
        else if (timeInPeriod < 7000) 
        {
            servoPositionDegrees = map(timeInPeriod, 6000, 7000, 180, 0);
        } 
        else if (timeInPeriod < 7500) 
        {
            servoPositionDegrees = map(timeInPeriod, 7000, 7500, 0, 180);
        } 
        else if (timeInPeriod < 8000) 
        {
            servoPositionDegrees = map(timeInPeriod, 7500, 8000, 180, 0);
        } 
        else 
        {
            servoPositionDegrees = 0;
        }

        gripperServo.write(servoPositionDegrees);
        updateFrame(frame, servoPositionDegrees);
        matrix.loadFrame(frame);

        // TODO: Check if WiFi is still connected (WiFi.status() == WL_CONNECTED) at regular intervals. If not, try to reconnect (Wifi.end() and connectToWifi());

    }
}


/**
 * Updates the frame that represents the 1-bit display buffer for the 12 x 8 pixel display.
 * Lights up from 0 to 12 pixels on the first two rows, depending on the servoPositionDegrees that
 * is from 0 to 180 degrees.
 */
void updateFrame(uint32_t *frame, int servoPositionDegrees) 
{
    // Check for valid servo position by clipping it to the range 0-180
    if (servoPositionDegrees < 0) 
    {
        servoPositionDegrees = 0;
    } 
    else if (servoPositionDegrees > 180) 
    {
        servoPositionDegrees = 180;
    }
    // Could do some error checking here on the frame buffer, but hey this is a hobby project.

    // Clear the frame first (reset all bits except type "TW" in the middle of the display))
    frame[0] = 0b00000000000000000000000001110100;
    frame[1] = 0b01000010010101000010001010000010;
    frame[2] = 0b00101000000000000000000000000000;
    
    // Map servo position (0-180) to number of pixels (0-12)
    int numPixels = map(servoPositionDegrees, 0, 180, 0, 12);
    
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

    Serial.println("\nStarting connection to server...");
    
    // TODO udp.begin(localNetworkPort);    
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