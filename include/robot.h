#ifndef ROBOT_H
#define ROBOT_H

#include "vecmath.h"


class Robot {
private:
    double x, y, theta;  // current (x, y, theta) coordinates
    int ir[3];           // current values of IR sensors


    int Vel_To_Value(double vel) const;       // converts velocity to servo value
    int Sensor_To_Range(double ir_value) const; // converts IR value to distance in cm

public:
    Robot();   // constructor
    ~Robot();  // destructor

    void Servo(int servo_number, int velocity); // drive 1 servo-motor
    void Drive(int s1_vel, int s2_vel, int s3_vel); // drive 3 servo-motors
    void Vector_Drive(vector dir, double omega); // drive the whole robot
    void Stop(); // stop robot
    int IRDist(int ir_number); // get distance from IR sensor
    void Message(const char* str); // display text message
    void Message(int number); // display number message
    void Wait(int delay_ticks, int *interrupt); // interruptible wait
};

#endif // ROBOT_H
