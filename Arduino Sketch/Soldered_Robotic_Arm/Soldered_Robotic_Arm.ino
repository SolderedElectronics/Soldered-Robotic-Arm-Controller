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
#include <Servo.h>  // Include Arduino Servo library

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
struct robotPart *robotPartArray[] = {&robotClawRotation, &robotClaw, &robotBodyTop, &robotBodyMiddle, &robotBodyBottom, &robotBodyRotation};

// Pins connected to the potentiometers. A0 = CLAW Rotation, A1 = Claw Grip, A2 = Body Top, A3 = Body Middle, A6 = Body Bottom, A7 = Body Rotation
int analogPins[] = {A0, A1, A2, A3, A6, A7};

// Servo pins. D7 = CLAW Rotation, D10 = Claw Grip, D9 = Body Top, D6 = Body Middle, D4 = Body Bottom, D5 = Body Rotation
int servoPins[] = {7, 10, 9, 6, 4, 5};

// NOTE! Pin order in array and struct array must not be changed! robotPartArray[0] is robotClawRotation where A0 is the analog input and D7 is servo output (note that in all arrays this is the first element).

void setup()
{
    // Init whole setup (analog pins, servo pins, max and min angles, etc)
    initSetup(robotPartArray, servoPins, analogPins, 6);
}

void loop()
{
    // Get ADC values of all potentiometers.
    getAnalogValues(robotPartArray, 6);

    // Map the ADC value into angle (using angleMin and angleMax of each motor defined in the struct)
    mapRawValues(robotPartArray, 6);

    // Set new angle of the servo motor.
    setServoValues(robotPartArray, 6);

    // Wait a little bit.
    delay(10);
}

void initSetup(struct robotPart **_s, int *_servoPins, int *_potPins, int _n)
{
    // First init structure for servo motor defines
    initStructs(_s, _servoPins, _potPins, _n);

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
    // Init all data in the struct for the motor. Set default max angle, min angle, analog input pin, servo output pin, and default angle and ADC values (just in case).
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
