#include <Arduino.h>
#include <ESP32MX1508.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "OmniDrive.h"

// Motor pins
const int motor1A = 4;
const int motor1B = 16;
const int motor2A = 21;
const int motor2B = 19;
const int motor3A = 27;
const int motor3B = 26;

// Encoder pins
const int encoder1A = 15;   //yellow
const int encoder1B = 2;    //white
const int encoder2A = 22;
const int encoder2B = 23;
const int encoder3A = 13;
const int encoder3B = 12;


const float wheelRadius = 0.062; // Example radius in meters 62mm


int delayTime = 2000;
int currentTime = 0;


OmniDrive omniDrive(motor1A, motor1B, motor2A, motor2B, motor3A, motor3B, wheelRadius);


MX1508 MotorL(0, 0, 0, 0);

void setup() {

    omniDrive.begin(encoder1A, encoder1B, encoder2A, encoder2B, encoder3A, encoder3B);


    omniDrive.setPIDTunings(1.0, 0.1, 0.01); //PID Vals
}

void loop() {

    currentTime = millis();
    if (currentTime < delayTime) {         //Vx = 1.0 m/s, Vy = 0, omega = 0 rad/s
        omniDrive.moveRobot(1.0, 0, 0);
    } else if (currentTime < 2 * delayTime) { //Vx = 0, Vy = 1.0 m/s, omega = 0 rad/s
        omniDrive.moveRobot(0, 1.0, 0);
    } else if (currentTime < 3 * delayTime) { //Vx = -1.0 m/s, Vy = 0, omega = 0 rad/s
        omniDrive.moveRobot(-1.0, 0, 0);
    } else if (currentTime < 4 * delayTime) { //Vx = 0, Vy = -1.0 m/s, omega = 0 rad/s
        omniDrive.moveRobot(0, -1.0, 0);
    } else if (currentTime < 5 * delayTime) {   //rotation 2 rad/s
        omniDrive.moveRobot(0, 0, 2.0);
    } else if (currentTime < 6 * delayTime) {   //rotation -2 rad/s
        omniDrive.moveRobot(0, 0, -2.0);
    } else if (currentTime < 7 * delayTime){    //reset robot and stop moving
        currentTime = 0;
        omniDrive.stop();
    } else currentTime = 0; // Stop robot in error
    delay(100);    

}
