/**
 * @brief Red Robot 2026 main code!
 * 
 * Read the comments below to figure out how to get started!
 */

// Libraries with driver code (don't remove these!)
#include <Arduino.h>                // make sure this is first
#include "RedRobot.h"



struct{
    float previous_error;
    float correction;
} PID_struct;

// PID tuning parameters
float Kp = 5.0f;
float Kd = 2.0f;

// Store error between iterations
float previous_error = 0.0f;

// Base motor speed
float current_speed = 30.0f;

// Declare some global variables
bool started; // boolean (true or false) to keep track of if we should start
int counter;  // integer to hold a count value

PID_struct PID_info = malloc(PID_struct);
PID_info->previous_error = previous_error;

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

void PID_calc(uint16_t array_of_sensor_vals, struct PID_struct *PID_info) {
    /* STEP 1: Calculate weighted average position */

    float weighted_sum = 0.0f;
    float total_darkness = 0.0f;

    for (int i = 0; i < 6; i++) {

        // Assumes 0 = black, 1000 = white
        float darkness = 1000.0f - array_of_sensor_vals[i];

        // Keep darkness nonnegative
        darkness = std::max(0.0f, darkness);

        // Sensor positions: -2.5 to +2.5 (ideally center is between 3 and 4 which corresponds to indexes 2 and 3)
        float position = i - 2.5f;

        weighted_sum += darkness * position;
        total_darkness += darkness;
    }

    /* STEP 2: Calculate error */

    // Stop if the line is not detected (error case which should never happen)
    if (total_darkness <= 0.0f) {
        RedRobot::WriteDCMotor(2, 0);
        RedRobot::WriteDCMotor(3, 0);
        return;
    }

    float error = weighted_sum / total_darkness;

    /* STEP 3: Calculate PD correction */

    float P = Kp * error;
    float D = Kd * (error - PID_info->previous_error);

    float correction = P + D;

    // IMPORTANT: Save error for next iteration
    previous_error = error;

    PID_info->correction = correction;
    PID_info->previous_error = previous_error;
}

void line_follow() {
    uint16_t sensor_values[6];
    RedRobot::ReadLineSensor(sensor_values);

    PID_calc(&sensor_values, &PID_info);

    /* STEP 4: Adjust motor speeds */

    float left_speed = current_speed + PID_info->correction;
    float right_speed = current_speed - PID_info->correction;

    // IMPORTANT: Check actual RedRobot motor speed limits
    left_speed = std::clamp(left_speed, -100.0f, 100.0f);
    right_speed = std::clamp(right_speed, -100.0f, 100.0f);

    /* STEP 5: Set servos */
    //if using servos you'll not want to reset them to 0 every time
    RedRobot::SetServo(0, 0);
    RedRobot::SetServo(1, 0);
    RedRobot::SetServo(2, 0);
    RedRobot::SetServo(3, 0);

    /* STEP 6: Move motors */

    RedRobot::WriteDCMotor(2, left_speed);
    RedRobot::WriteDCMotor(3, right_speed);
}

// This function runs over and over (in a loop) when you are in autonomous mode.
// As an example, this code moves out of the starting box at start, and 
// Look at RedRobot.h to see the robot interface: what functions you can call and how.

void AutonomousLoop() {
    unsigned long initialTimems = 0;
    
    //auton_state is if we are in scoring or 
    if (auton_state == 1) {
        auton_button();
    } else {
        auton_score();
    }
}
//direction_state is L/R maybe u set globals idk thats aarna's function here

/*
AUTON OPTIONS
GOAL: BUTTON
start (L/R)
line follow
run into wall 
done (enter teleop)

GOAL: SCORE
start (L/R)
line follow (forward really)
lined up
dispense
done (enter teleop)
*/

// GOAL: BUTTON //
//left and right are the same bc no components are moving
/* PUT THE FOLLOWING IN THE LOOP BEFORE CALLING THIS FUNCTION
unsigned long initialTimems = 0; 
*/
void auton_button() {
    //line follow
    unsigned long currentTimems = millis(); //in ms

    //change the first val to the amount of seconds, 100 makes in ms
    while ((initialTimems - currentTimems) < (10*100)) {
        lineFollow();
    }

    //hit the wall
    //stop moving forward after an amount of time but that will be determined after we can do some testing
}

// GOAL: SCORE //
//left and right are the same bc no components are moving
/* PUT THE FOLLOWING IN THE LOOP BEFORE CALLING THIS FUNCTION
unsigned long initialTimems = 0; 
*/
void auton_score() {
    //line follow
    unsigned long currentTimems = millis(); //in ms

    //change the first val to the amount of seconds, 100 makes in ms
    //this one the amount of time does matter, how long until we're at the zone so lets spit it out
    while ((initialTimems - currentTimems) < (10*100)) {
        lineFollow();
    }
    //lined up? wait it will be lol

    //dispense
    //aka reverse direction of the intake wheel
    //RedRobot::write to that yellow motor to go the negative (out) direction
    RedRobot::WriteDCMotor(0, -100); //CHECK THIS IDK HOW TO DO THE YELLOW ONE
}

