/**
 **************************************************
 *
 * @file        Soldered_robotic_arm.ino
 * @brief       Control the robotic arm from Soldered with this simple code.
 *              Move sliders to control the movement of the arm.
 *              There are total of 6 degrees of freedom.
 *              For his project you will need:
 *              - 6 x Soldered Slider Potentiometer: www.solde.red/333130
 *              - 6 x MG995 Servo motor: www.solde.red/101388
 *              - Robotic arm: www.solde.red/108995
 *              - Dasduino Core: www.solde.red/333037
 *              - Good 5V power supply (at least 5A): www.solde.red/100355
 *              - Wires (red, yellow, black): www.solde.red/108627, www.solde.red/108967,
 *              www.solde.red/108626
 *              - Headres: www.solde.red/101172
 *              - Heatshrink: www.solde.red/000029
 *
 *              To use Dasduino Core, Arduino Dasduino Core needs to be installed.
 *
 * @note        Be careful using provided power supply.
 *              Working with live 240VAC can be lethal!!!
 *
 * @authors     borna@soldered.com
 ***************************************************/
#include "WS2812-SOLDERED.h" // Include Soldered WS2812 LED Library
#include <Servo.h>           // Include Arduino Servo library

// Input pin for mode select switch (active or resting robotic arm position)
#define MODE_SWITCH 11

// Output pin for WS2812 LED showing the status of the device (red -> resting, green -> active)
#define MODE_LED 12

// Stuct for defining motor parameters.
struct robotPart
{
    uint8_t angle;
    uint8_t angleMax;
    uint8_t angleMin;
    uint16_t rawAdcValue;
    int analogPin;
    int servoPin;
    Servo servo;
};

// Struct for defining every motor on the robot.
struct robotPart robotClawRotation;
struct robotPart robotClaw;
struct robotPart robotBodyTop;
struct robotPart robotBodyMiddle;
struct robotPart robotBodyBottom;
struct robotPart robotBodyRotation;

// Array of structures for easy accessing each motor
struct robotPart *robotPartArray[] = {&robotClawRotation, &robotClaw,       &robotBodyTop,
                                      &robotBodyMiddle,   &robotBodyBottom, &robotBodyRotation};

// robotPartArray[0] -> robotClawRotation
// robotPartArray[1] -> robotClaw
// robotPartArray[2] -> robotBodyTop
// robotPartArray[3] -> robotBodyMiddle
// robotPartArray[4] -> robotBodyBottom
// robotPartArray[5] -> robotBodyRotation

// Pins connected to the potentiometers. A0 = CLAW Rotation, A1 = Claw Grip, A2 = Body Top, A3 = Body Middle, A6 = Body
// Bottom, A7 = Body Rotation
int analogPins[] = {A0, A1, A2, A3, A6, A7};

// Servo pins. D7 = CLAW Rotation, D10 = Claw Grip, D9 = Body Top, D6 = Body Middle, D4 = Body Bottom, D5 = Body
// Rotation
int servoPins[] = {7, 10, 9, 6, 4, 5};

// NOTE! Pin order in array and struct array must not be changed! robotPartArray[0] is robotClawRotation where A0 is the
// analog input and D7 is servo output (note that in all arrays this is the first element).

// Object for WS2812 LED
WS2812 led(1, MODE_LED);

// Variable for the switch state
int switchState;

void setup()
{
    Serial.begin(115200);

    // Init whole setup (analog pins, servo pins, max and min angles, etc)
    initSetup(robotPartArray, servoPins, analogPins, 6);
}

void loop()
{
    // Read the mode switch state
    int newSwitchState = digitalRead(MODE_SWITCH);

    // Check if the switch state has chaneg.
    if (newSwitchState != switchState)
    {
        setLedColor(&led, newSwitchState);
        if (!newSwitchState)
        {
            setNormalPosition(robotPartArray, 6);
        }
        else
        {
            setRestPosition(robotPartArray, 6);
        }
        switchState = newSwitchState;
    }

    if (!newSwitchState)
    {
        // Get ADC values of all potentiometers.
        getAnalogValues(robotPartArray, 6);

        // Map the ADC value into angle (using angleMin and angleMax of each motor defined in the struct)
        mapRawValues(robotPartArray, 6);

        // Set new angle of the servo motor.
        setServoValues(robotPartArray, 6);
    }

    // Wait a little bit.
    delay(100);
}

void initSetup(struct robotPart **_s, int *_servoPins, int *_potPins, int _n)
{
    // First init structure for servo motor defines
    initStructs(_s, _servoPins, _potPins, _n);

    // Init input pin for mode select switch
    pinMode(MODE_SWITCH, INPUT_PULLUP);

    // Init WS2812 library
    led.begin();

    // Set LED color accordingly with switch state
    switchState = digitalRead(MODE_SWITCH);
    setLedColor(&led, switchState);

    // Init input pins (for potentiometers)
    initAnalogPins(_s, _n);

    // Init servo pins
    initServo(_s, _n);

    // Set custom angle limits for each motor
    // Change not needed
    robotBodyRotation.angleMin = 0;
    robotBodyRotation.angleMax = 180;

    // Flip it
    robotBodyBottom.angleMin = 180;
    robotBodyBottom.angleMax = 0;

    // Flip it
    robotBodyMiddle.angleMin = 180;
    robotBodyMiddle.angleMax = 0;

    // Change not needed
    robotBodyTop.angleMin = 0;
    robotBodyTop.angleMax = 180;

    // Change not needed
    robotClawRotation.angleMin = 0;
    robotClawRotation.angleMax = 180;

    // Flip it and limit the angle.
    robotClaw.angleMin = 130;
    robotClaw.angleMax = 66;
}

void initAnalogPins(struct robotPart **_s, int _n)
{
    // Set all pins to be input for the potentiometer.
    for (int i = 0; i < _n; i++)
    {
        pinMode(_s[i]->analogPin, INPUT);
    }
}

void initStructs(struct robotPart **_s, int *_servoPins, int *_potPins, int _n)
{
    // Init all data in the struct for the motor. Set default max angle, min angle, analog input pin, servo output pin,
    // and default angle and ADC values (just in case).
    for (int i = 0; i < _n; i++)
    {
        _s[i]->angleMax = 180;
        _s[i]->angleMax = 0;
        _s[i]->angle = 90;
        _s[i]->rawAdcValue = 512;
        _s[i]->analogPin = _potPins[i];
        _s[i]->servoPin = _servoPins[i];
    }
}

void initServo(struct robotPart **_s, int _n)
{
    // Ser all servo pins to output, low
    for (int i = 0; i < _n; i++)
    {
        pinMode(_s[i]->servoPin, OUTPUT);
        digitalWrite(_s[i]->servoPin, LOW);
    }

    // Attach pin to a servo object
    for (int i = 0; i < _n; i++)
    {
        _s[i]->servo.attach(_s[i]->servoPin);
    }
}

void getAnalogValues(struct robotPart **_s, int _n)
{
    // Enable ADC Noise Reduction Mode on ATMEGA328P
    SMCR = (0 << SM2) | (0 << SM1) | (1 << SM0) | (1 << SE);

    // Read analog value for each potentiometer and store it in struct.
    for (int i = 0; i < _n; i++)
    {
        _s[i]->rawAdcValue = analogRead(_s[i]->analogPin);
    }

    // Disable ADC Noise reduction mode
    SMCR = 0;
}

void mapRawValues(struct robotPart **_s, int _n)
{
    // Convert ADC vaule into angle value for each motor.
    for (int i = 0; i < _n; i++)
    {
        _s[i]->angle = map(_s[i]->rawAdcValue, 0, 1023, _s[i]->angleMin, _s[i]->angleMax);
    }
}

void setServoValues(struct robotPart **_s, int _n)
{
    // Send newly calculated angle values to the motors.
    for (int i = 0; i < _n; i++)
    {
        _s[i]->servo.write(_s[i]->angle);
    }
}

void setRestPosition(struct robotPart **_s, int _n)
{
    // Angles of the servo motors wheh robotic arm is in the rest position.
    uint8_t robotClawRotation = 51;
    uint8_t robotClaw = 105;
    uint8_t robotBodyTop = 24;
    uint8_t robotBodyMiddle = 55;
    uint8_t robotBodyBottom = 43;
    uint8_t robotBodyRotation = 70;

    // Now each motor needs to go to from the current location to the rest position

    // First release everything from the claw
    moveHandIntoPosition(_s, 1, _s[1]->angle, _s[1]->angleMax);

    // Move claw into rest position
    moveHandIntoPosition(_s, 1, _s[1]->angle, robotClaw);

    // Next claw rotation
    moveHandIntoPosition(_s, 0, _s[0]->angle, robotClawRotation);

    // Rotate whole arm
    moveHandIntoPosition(_s, 5, _s[5]->angle, robotBodyRotation);

    // Move top body
    moveHandIntoPosition(_s, 2, _s[2]->angle, robotBodyTop);

    // Move middle mody
    moveHandIntoPosition(_s, 3, _s[3]->angle, robotBodyMiddle);

    // At last move bottom
    moveHandIntoPosition(_s, 4, _s[4]->angle, robotBodyBottom);

    // _Detach all servo motors
    for (int i = 0; i < _n; i++)
    {
        _s[i]->servo.detach();
    }
}

void moveHandIntoPosition(struct robotPart **_s, int _motor, uint8_t _oldPosition, uint8_t _newPositon)
{
    // Variable for direciton
    int _dir;

    // If it's already into place, there is no need for movement
    if (_oldPosition == _newPositon)
        return;

    // Check if angle needs to be incremented or decremented
    if (_oldPosition > _newPositon)
    {
        _dir = -1;
    }
    else
    {
        _dir = 1;
    }

    // Move arm into new position.
    while (_oldPosition != _newPositon)
    {
        _oldPosition += _dir;
        _s[_motor]->servo.write(_oldPosition);
        delay(50);
    }

    _s[_motor]->angle = _newPositon;
}

void setNormalPosition(struct robotPart **_s, int _n)
{
    // Save current position of each motor
    uint8_t robotClawRotation = _s[0]->angle;
    uint8_t robotClaw = _s[1]->angle;
    uint8_t robotBodyTop = _s[2]->angle;
    uint8_t robotBodyMiddle = _s[3]->angle;
    uint8_t robotBodyBottom = _s[4]->angle;
    uint8_t robotBodyRotation = _s[5]->angle;

    // Attach all motors again
    initServo(_s, 6);

    // Read current position of the potentiometers and fill the struct
    // Get ADC values of all potentiometers.
    getAnalogValues(robotPartArray, 6);

    // Map the ADC value into angle (using angleMin and angleMax of each motor defined in the struct)
    mapRawValues(robotPartArray, 6);

    // First move bottom
    moveHandIntoPosition(_s, 4, robotBodyBottom, _s[4]->angle);

    // Move middle mody
    moveHandIntoPosition(_s, 3, robotBodyMiddle, _s[3]->angle);

    // Move top body
    moveHandIntoPosition(_s, 2, robotBodyTop, _s[2]->angle);

    // Rotate whole arm
    moveHandIntoPosition(_s, 5, robotBodyRotation, _s[5]->angle);

    // Next claw rotation
    moveHandIntoPosition(_s, 0, robotClawRotation, _s[0]->angle);

    // Move claw into rest position
    moveHandIntoPosition(_s, 1, robotClaw, _s[1]->angle);
}

void setLedColor(WS2812 *_l, uint8_t _mode)
{
    _l->setPixelColor(0, _mode ? _l->Color(0, 255, 0) : _l->Color(255, 0, 0));
    _l->show();
}

// Print out current angles (for debug purpose)
void printAngleStatus(struct robotPart **_s)
{
    // {&robotClawRotation, &robotClaw, &robotBodyTop, &robotBodyMiddle, &robotBodyBottom, &robotBodyRotation};
    Serial.print("robotClawRotation: ");
    Serial.println(_s[0]->angle, DEC);
    Serial.print("robotClaw: ");
    Serial.println(_s[1]->angle, DEC);
    Serial.print("robotBodyTop: ");
    Serial.println(_s[2]->angle, DEC);
    Serial.print("robotBodyMiddle: ");
    Serial.println(_s[3]->angle, DEC);
    Serial.print("robotBodyBottom: ");
    Serial.println(_s[4]->angle, DEC);
    Serial.print("robotBodyRotation: ");
    Serial.println(_s[5]->angle, DEC);
}