/**
 * @brief Red Robot 2026 main code!
 * 
 * Read the comments below to figure out how to get started!
 */

// Libraries with driver code (don't remove these!)
#include <Arduino.h>                // make sure this is first
#include "RedRobot.h"



struct pid_struct {
    float previous_error;
    float correction;
};

// Explicit declaration keeps Arduino prototype generation after the type.
bool PID_calc(const uint16_t array_of_sensor_vals[6], pid_struct *state);
void line_follow();
void auton_button();
void auton_score();

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

pid_struct PID_info = {0.0f, 0.0f};
int auton_state = 1; // 1 = button route, 2 = scoring route
unsigned long initialTimems = 0;
const unsigned long FOLLOW_DURATION_MS = 10000UL;

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

bool PID_calc(const uint16_t array_of_sensor_vals[6], pid_struct *state) {
    /* STEP 1: Calculate weighted average position */

    float weighted_sum = 0.0f;
    float total_darkness = 0.0f;

    for (int i = 0; i < 6; i++) {

        // Assumes 0 = black, 1000 = white
        float darkness = 1000.0f - array_of_sensor_vals[i];

        // Keep darkness nonnegative
        darkness = max(0.0f, darkness);

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
        state->previous_error = 0.0f;
        state->correction = 0.0f;
        return false;
    }

    float error = weighted_sum / total_darkness;

    /* STEP 3: Calculate PD correction */

    float P = Kp * error;
    float D = Kd * (error - state->previous_error);

    float correction = P + D;

    // IMPORTANT: Save error for next iteration
    previous_error = error;

    state->correction = correction;
    state->previous_error = previous_error;
    return true;
}

void line_follow() {
    uint16_t sensor_values[6];
    RedRobot::ReadLineSensor(sensor_values);

    if (!PID_calc(sensor_values, &PID_info)) return;

    /* STEP 4: Adjust motor speeds */

    float left_speed = current_speed + PID_info.correction;
    float right_speed = current_speed - PID_info.correction;

    // IMPORTANT: Check actual RedRobot motor speed limits
    left_speed = constrain(left_speed, -100.0f, 100.0f);
    right_speed = constrain(right_speed, -100.0f, 100.0f);

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
    if (!started) {
        initialTimems = millis();
        started = true;
    }
    
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

// One line-following update per call keeps autonomous mode responsive.
void auton_button() {
    /*
    if (millis() - initialTimems < FOLLOW_DURATION_MS) {
        line_follow();
    } else {
        RedRobot::WriteDCMotor(2, 0);
        RedRobot::WriteDCMotor(3, 0);
    }
    */
    line_follow();
}

void auton_score() {
    /*
    if (millis() - initialTimems < FOLLOW_DURATION_MS) {
        line_follow();
        return;
    }
    RedRobot::WriteDCMotor(2, 0);
    RedRobot::WriteDCMotor(3, 0);
    // Existing scoring behavior: run the intake in reverse.
    RedRobot::WriteDCMotor(0, -100);
    */
    line_follow();
}
