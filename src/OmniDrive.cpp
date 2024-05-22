#include <ESP32MX1508.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "OmniDrive.h"
#include "vecmath.h"

// Constructor to initialize motor and PID instances
OmniDrive::OmniDrive(int pin1A, int pin1B, int pin2A, int pin2B, int pin3A, int pin3B, float radius)
    : _motor1(pin1A, pin1B, 0, 1), _motor2(pin2A, pin2B, 2, 3), _motor3(pin3A, pin3B, 4, 5),
      _pid1(&_input1, &_output1, &_setpoint1, 0, 0, 0, DIRECT),
      _pid2(&_input2, &_output2, &_setpoint2, 0, 0, 0, DIRECT),
      _pid3(&_input3, &_output3, &_setpoint3, 0, 0, 0, DIRECT),
      _lastTime(0), _prevEncoderCount1(0), _prevEncoderCount2(0), _prevEncoderCount3(0),
      _lastDebugTime(0), R(radius) {}

// Initialize encoders and PID controllers
void OmniDrive::begin(int enc1A, int enc1B, int enc2A, int enc2B, int enc3A, int enc3B)
{
    _encoder1.attachHalfQuad(enc1A, enc1B);
    _encoder2.attachHalfQuad(enc2A, enc2B);
    _encoder3.attachHalfQuad(enc3A, enc3B);
    _pid1.SetMode(AUTOMATIC);
    _pid2.SetMode(AUTOMATIC);
    _pid3.SetMode(AUTOMATIC);
    _lastTime = millis();
    Serial.begin(115200);
    Serial.println("OmniDrive initialized");
}

// Adjust motor output to ensure minimum motor output
int OmniDrive::adjustMotorOutput(int motorOutput, int _min) {
    if (motorOutput < 0) {
        return min(-_min, -motorOutput);
    } else {
        return max(_min, motorOutput);
    }
}

// Move the robot in a given direction with a specified angular velocity
void OmniDrive::moveRobot(float Vx, float Vy, float omega)
{
    // Create direction vector V from input velocities Vx and Vy
    vector V(Vx, Vy);

    // Define wheel orientation vectors
    vector F1(-1.000, 0.000);   // Vector pointing directly left
    vector F2(0.500, -0.866);  // Vector pointing 120 degrees counter-clockwise
    vector F3(0.866, 0.500);   // Vector pointing 240 degrees counter-clockwise

    // Robot-specific constants
    double b = 0.090; // Wheelbase
    double r = 0.020; // Wheel radius

    // Calculate wheel speeds using dot product of direction vector and wheel orientation vectors
    // Plus the contribution from angular velocity omega
    double omega1 = (F1 * V + b * omega) / r;
    double omega2 = (F2 * V + b * omega) / r;
    double omega3 = (F3 * V + b * omega) / r;

    // Check if the calculated wheel speeds are within the physical limits
    if ((omega1 > 6) || (omega2 > 6) || (omega3 > 6) || (omega1 < -6) || (omega2 < -6) || (omega3 < -6)) {
        Serial.println("Vectors: ERROR!");
        return;
    }

    // Set the desired speeds (setpoints) for PID controllers
    _setpoint1 = omega1;
    _setpoint2 = omega2;
    _setpoint3 = omega3;

    // Get the current time and calculate elapsed time
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - _lastTime) / 1000.0; // Convert milliseconds to seconds
    _lastTime = currentTime;

    // Read encoder counts to get the current position of each wheel
    long encoderCount1 = _encoder1.getCount();
    long encoderCount2 = _encoder2.getCount();
    long encoderCount3 = _encoder3.getCount();

    // Calculate wheel speeds in counts per second
    float speed1 = (encoderCount1 - _prevEncoderCount1) / deltaTime;
    float speed2 = (encoderCount2 - _prevEncoderCount2) / deltaTime;
    float speed3 = (encoderCount3 - _prevEncoderCount3) / deltaTime;

    // Update previous encoder counts with the current counts
    _prevEncoderCount1 = encoderCount1;
    _prevEncoderCount2 = encoderCount2;
    _prevEncoderCount3 = encoderCount3;

    // Set PID inputs to the calculated wheel speeds
    _input1 = speed1;
    _input2 = speed2;
    _input3 = speed3;

    // Compute PID outputs to adjust motor speeds
    _pid1.Compute();
    _pid2.Compute();
    _pid3.Compute();

    // Constrain PID outputs to be within motor input range (-255 to 255)
    int motorOutput1 = constrain(_output1, -255, 255);
    int motorOutput2 = constrain(_output2, -255, 255);
    int motorOutput3 = constrain(_output3, -255, 255);

    // Adjust motor outputs to account for min motor torque required to move the robot
    motorOutput1 = (motorOutput1 == 0) ? 0 : adjustMotorOutput(motorOutput1, 50); // 50 is the minimum motor output
    motorOutput2 = (motorOutput2 == 0) ? 0 : adjustMotorOutput(motorOutput2, 50);
    motorOutput3 = (motorOutput3 == 0) ? 0 : adjustMotorOutput(motorOutput3, 50);

    // Drive the motors with the adjusted outputs
    _motor1.motorGo(motorOutput1);
    _motor2.motorGo(motorOutput2);
    _motor3.motorGo(motorOutput3);

    // Print debug info every 0.5 seconds
    if (currentTime - _lastDebugTime >= 500)
    {
        printDebugInfo();
        _lastDebugTime = currentTime;
    }
}

// Stop the robot by braking all motors
void OmniDrive::stop()
{
    _motor1.motorBrake();
    _motor2.motorBrake();
    _motor3.motorBrake();
    Serial.println("Robot stopped");
}

// Set PID tunings for all motors
void OmniDrive::setPIDTunings(double kp, double ki, double kd)
{
    _pid1.SetTunings(kp, ki, kd);
    _pid2.SetTunings(kp, ki, kd);
    _pid3.SetTunings(kp, ki, kd);
    Serial.print("PID Tunings set: ");
    Serial.print("Kp=");
    Serial.print(kp);
    Serial.print(", Ki=");
    Serial.print(ki);
    Serial.print(", Kd=");
    Serial.println(kd);
}

// Get or reset encoder counts
int OmniDrive::encoderCounts(int encoder, bool readWrite)
{
    switch (encoder)
    {
    case 1:
        if (readWrite)
        {
            _encoder1.setCount(0);
        }
        else
        {
            return _encoder1.getCount();
        }
        break;
    case 2:
        if (readWrite)
        {
            _encoder2.setCount(0);
        }
        else
        {
            return _encoder2.getCount();
        }
        break;
    case 3:
        if (readWrite)
        {
            _encoder3.setCount(0);
        }
        else
        {
            return _encoder3.getCount();
        }
        break;
    default:
        return -1;
        break;
    }
}

// Print debug information to the serial monitor
void OmniDrive::printDebugInfo()
{
    Serial.print("Setpoints: ");
    Serial.print(_setpoint1);
    Serial.print(", ");
    Serial.print(_setpoint2);
    Serial.print(", ");
    Serial.println(_setpoint3);

    Serial.print("Speeds: ");
    Serial.print(_input1);
    Serial.print(", ");
    Serial.print(_input2);
    Serial.print(", ");
    Serial.println(_input3);

    Serial.print("Outputs: ");
    Serial.print(_output1);
    Serial.print(", ");
    Serial.print(_output2);
    Serial.print(", ");
    Serial.println(_output3);

    Serial.print("Motor outputs: ");
    Serial.print(constrain(_output1, -255, 255));
    Serial.print(", ");
    Serial.print(constrain(_output2, -255, 255));
    Serial.print(", ");
    Serial.println(constrain(_output3, -255, 255));
}
