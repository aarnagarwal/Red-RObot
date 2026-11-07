/**
 * @brief Red Robot 2026 main code!
 * 
 * Read the comments below to figure out how to get started!
 */

// Libraries with driver code (don't remove these!)
#include <Arduino.h>                // make sure this is first
#include "RedRobot.h"

// Declare some global variables
bool started; // boolean (true or false) to keep track of if we should start
int counter;  // integer to hold a count value

// This function runs ONCE, when the microcontroller boots up
void setup() {
    // Initialize all components on the robot before doing any work
  RedRobot::InitRedRobot();         // call this first
  RedRobot::InitDCMotors();
  RedRobot::InitServo();
  RedRobot::InitLineSensors();
  RedRobot::InitAuton();            // call this to activate the AutonomousLoop

  // Initialize our variables
  started = false;
  counter = 0;
}

// This function runs over and over (in a loop) when you are in autonomous mode.
// As an example, this code moves out of the starting box at start, and 
// Look at RedRobot.h to see the robot interface: what functions you can call and how.

float current_speed = 30;

void AutonomousLoop() {
    // move forward - set motors 2 and 3 to speed 30
    RedRobot::WriteDCMotor(2, current_speed);
    RedRobot::WriteDCMotor(3, current_speed);

    uint16_t sensor_values[6];
    RedRobot:ReadLineSensor(sensor_values);

    /*     PID stuff     */
    // STEP 1: Calculate weighted average position
    float weighted_sum = 0.0f;
    float total_darkness = 0.0f;

    for (int i = 0; i < 6; i++) {
        // give more importance 
        float darkness = 1000.0f - sensor_values[i];

        // Sensor positions: -2.5, -1.5, -0.5,
        //                    +0.5, +1.5, +2.5
        float position = i - 2.5f;

        weighted_sum += darkness * position;
        total_darkness += darkness;
    }

    float P = Kp * error;
    float D = Kd * (error - previous_error);

    float correction = P + D;
    /* PID calculations ended */

    float left_speed = current_speed + correction;
    float right_speed = current_speed - correction;

    // Keep motor speeds in valid range
    left_speed  = std::clamp(left_speed, 0.0f, 255.0f);
    right_speed = std::clamp(right_speed, 0.0f, 255.0f);

    // set all servos to angle 0
    RedRobot::SetServo(0, 0);
    RedRobot::SetServo(1, 0);
    RedRobot::SetServo(2, 0);
    RedRobot::SetServo(3, 0);

    // move forward at adjustment speed
    RedRobot::WriteDCMotor(2, left_speed);
    RedRobot::WriteDCMotor(3, right_speed);
}
