/**
 * Controlling a TowerPro MG996R servo,
 * using an Arduino Uno R4 WiFi. 
 * 
 * Servo specs:
 * ------------
 * Weight: 55g
 * Dimension: 40.7×19.7×42.9mm
 * Stall torque: 9.4kg/cm (4.8v); 11kg/cm (6.0v)
 * Operating speed: 0.19sec/60degree (4.8v); 0.15sec/60degree (6.0v)
 * Operating voltage: 4.8 ~ 6.6v
 * Current draw at idle 10mA
 * No load operating current draw 170mA
 * Stall current draw 1400mA
 * 
 * Wiring:
 * Brown: GND
 * Red: VCC (4.8 ~ 6.6v)
 * Orange: PWM Signal
 */

#include "Arduino_LED_Matrix.h"
#include <Servo.h>


const int16_t SERVO_PIN = 9;

Servo gripper; 
int16_t servoPosition = 0;

ArduinoLEDMatrix matrix;
uint32_t frame[] =
{
    0x00000000,
    0x00000000,
    0x00000000
};
const uint32_t happy[] = 
{
    0x19819,
    0x80000001,
    0x81f8000
};
const uint32_t heart[] =
{
    0x3184a444,
    0x44042081,
    0x100a0040
};



void setup() 
{
    Serial.begin(115200);
    matrix.begin();
    gripper.attach(SERVO_PIN);
}

  
void loop() 
{
    matrix.loadFrame(heart);
    delay(250);

    matrix.loadFrame(happy);
    delay(250);

    while(true)
    {
        uint32_t milliseconds = millis();
        uint32_t timeInPeriod = milliseconds % 10000; // 10 seconds cycle

        if (timeInPeriod < 2000) 
        {
            servoPosition = map(timeInPeriod, 0, 2000, 0, 180);
        } 
        else if (timeInPeriod < 4000) 
        {
            servoPosition = map(timeInPeriod, 2000, 4000, 180, 0);
        } 
        else if (timeInPeriod < 5000) 
        {
            servoPosition = 0;
        } 
        else if (timeInPeriod < 6000) 
        {
            servoPosition = map(timeInPeriod, 5000, 6000, 0, 180);
        } 
        else if (timeInPeriod < 7000) 
        {
            servoPosition = map(timeInPeriod, 6000, 7000, 180, 0);
        } 
        else if (timeInPeriod < 7500) 
        {
            servoPosition = map(timeInPeriod, 7000, 7500, 0, 180);
        } 
        else if (timeInPeriod < 8000) 
        {
            servoPosition = map(timeInPeriod, 7500, 8000, 180, 0);
        } 
        else 
        {
            servoPosition = 0;
        }

        gripper.write(servoPosition);
        updateFrame(frame, servoPosition);
        matrix.loadFrame(frame);

    }
}


/**
 * Updates the frame that represents the 1-bit display buffer for the 12 x 8 pixel display.
 * Lights up from 0 to 12 pixels on the first two rows, depending on the servoPosition that
 * is from 0 to 180 degrees.
 */
void updateFrame(uint32_t *frame, int servoPosition) {
    // Could do some error checking here on the parameters, but hey this is a hobby project - we assume valid input.

    // Clear the frame first (reset all bits except type "TW" in the middle of the display))
    frame[0] = 0b00000000000000000000000001110100;
    frame[1] = 0b01000010010101000010001010000010;
    frame[2] = 0b00101000000000000000000000000000;
    
    // Map servo position (0-180) to number of pixels (0-12)
    int numPixels = map(servoPosition, 0, 180, 0, 12);
    
    // Set pixels in first two rows
    for (int bitPosition = 0; bitPosition < numPixels; bitPosition++) {
        // Set pixel in first row (bits 31-20)
        frame[0] |= (1UL << (31 - bitPosition));
        // Set pixel in second row (bits 19-8)
        frame[0] |= (1UL << (19 - bitPosition));
    }
}
