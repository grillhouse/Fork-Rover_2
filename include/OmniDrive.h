#ifndef OmniDrive_h
#define OmniDrive_h

#include <ESP32MX1508.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "vecmath.h"

class OmniDrive
{
public:
    OmniDrive(int pin1A, int pin1B, int pin2A, int pin2B, int pin3A, int pin3B, float radius);
    void begin(int enc1A, int enc1B, int enc2A, int enc2B, int enc3A, int enc3B);
    void moveRobot(float Vx, float Vy, float omega);
    void stop();
    void setPIDTunings(double kp, double ki, double kd);
    int encoderCounts(int encoder, bool readWrite);

private:
    MX1508 _motor1;
    MX1508 _motor2;
    MX1508 _motor3;
    ESP32Encoder _encoder1;
    ESP32Encoder _encoder2;
    ESP32Encoder _encoder3;
    PID _pid1;
    PID _pid2;
    PID _pid3;
    double _setpoint1, _input1, _output1;
    double _setpoint2, _input2, _output2;
    double _setpoint3, _input3, _output3;
    long _prevEncoderCount1, _prevEncoderCount2, _prevEncoderCount3;
    unsigned long _lastTime;
    float R; // Radius of the robot
    void printDebugInfo();
    int adjustMotorOutput(int motorOutput, int _min);

    unsigned long _lastDebugTime;
};

#endif
