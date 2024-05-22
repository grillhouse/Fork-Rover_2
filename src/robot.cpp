#include "robot.h"
#include <cmath>


// Calculates distance using Power Regression y=a*x^b
// returns value in cm ranging from 10 to 80
int Robot::Sensor_To_Range(double x) const {
    double a = 2141.72055;
    double b = -1.078867;
    int y = 100;
    if ((x > 0) && (x < 250)) {
        y = a * pow(x, b);
    }
    if (y < 10) y = 10;
    if (y > 100) y = 100;
    return y;
}

// Calculates servo value using Cubic Regression y=a*x^3+b*x^2+c*x+d
// Converts velocity in rad per second to servo value (0-255)
// vel can only be [-6,6] rad/sec
// x can only be [-1,1] rev/sec
int Robot::Vel_To_Value(double vel) const {
    double x = vel / PI / 2;  // convert rad/sec to rev/sec
    if ((vel > -6) && (vel < 6)) {
        return (39.8053 * x * x * x - 12.6083 * x * x + 22.1197 * x + 128.4262);
    } else {
        return 0;
    }
}

// default constructor
Robot::Robot() {
    // Err error;  // find and open MathLib library
    // error = SysLibFind(MathLibName, &MathLibRef);
    // if (error) error = SysLibLoad(LibType, MathLibCreator, &MathLibRef);
    // ErrFatalDisplayIf(error, "Can't find MathLib");
    // error = MathLibOpen(MathLibRef, MathLibVersion);
    // ErrFatalDisplayIf(error, "Can't open MathLib");
    // Stop();  // stop all motors
}

// default destructor
Robot::~Robot() {
    // Err error;  // close MathLib library
    // UInt usecount;
    // error = MathLibClose(MathLibRef, &usecount);
    // ErrFatalDisplayIf(error, "Can't close MathLib");
    // if (usecount == 0) SysLibRemove(MathLibRef);
    // Stop();  // stop all motors
}

void Robot::Servo(int servo_number, int velocity) {
    // board.Tx("SV", servo_number);
    // board.Tx("M", velocity);
}

// drive 3 servo-motors
void Robot::Drive(int s1_vel, int s2_vel, int s3_vel) {
    Servo(1, s1_vel);
    Servo(2, s2_vel);
    Servo(3, s3_vel);
}

// Drives robot in a given direction vector V with given angular
// velocity omega.
// r - wheel radius in meters
// b - wheel baseline in meters
// ||V|| must be less than ~0.1 !
void Robot::Vector_Drive(vector V, double omega) {
    vector F1(-1.000, 0.000), F2(0.500, -0.866), F3(0.866, 0.500);
    double omega1, omega2, omega3, b = 0.090, r = 0.020;
    omega1 = (F1 * V + b * omega) / r;  // F1*V is overloaded dot product
    omega2 = (F2 * V + b * omega) / r;
    omega3 = (F3 * V + b * omega) / r;
    // makes sure that given path is physically possible
    if ((omega1 > 6) || (omega2 > 6) || (omega3 > 6) || (omega1 < -6) || (omega2 < -6) || (omega3 < -6)) {
        Message("Vectors: ERROR!");
    } else {
        Drive(Vel_To_Value(omega1), Vel_To_Value(omega2), Vel_To_Value(omega3)) ;
    }
}



// stop each servo motor
void Robot::Stop() {
    Servo(1, 0);
    Servo(2, 0);
    Servo(3, 0);
}

// reads and returns distance in cm from IR sensor to an obstacle
int Robot::IRDist(int ir_number) {
    // int value = 0, distance;
    // char incoming[] = "999";
    // board.Tx("AD", ir_number);
    // board.Rc(incoming);
    // // buffer correction -- replace all ASCII non-numbers with spaces
    // for (int l = 0; l <= 2; l++)
    //     if (incoming[l] == 10)
    //         incoming[l] = ' ';
    // incoming[3] = '\0';
    // if (incoming[0] != '1')  // two digit number cut-off
    //     incoming[2] = '\0';
    // value = StrAToI(incoming);
    // distance = Sensor_To_Range(value);
    // return distance;
}

// Pauses application for specified number of ticks
// Can be interrupted by penDown event
// If *interrupt was provided:
// Sets *interrupt to 1 if it was interrupted by penDown
// Sets *interrupt to 0 if delay ended with no interrupts
void Robot::Wait(int delay_ticks, int *interrupt) {
    // ULong start_time;
    // EventPtr quit_event;
    // start_time = TimGetTicks();
    // while (TimGetTicks() < (start_time + delay_ticks)) {
    //     EvtGetEvent(quit_event, 1);  // get event using 1ms timeout
    //     if (quit_event->eType == penDownEvent) { // stop if screen was touched anywhere
    //         if (interrupt != NULL) {
    //             *interrupt = 1;
    //         }
    //         break;
    //     }
    // }
    // if (interrupt != NULL) {
    //     *interrupt = 0;
    // }
}
