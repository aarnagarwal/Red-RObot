const int numLine = 5;
const int distVal = 13;
#include <Servo.h>

#include <NewPing.h>
const int LEFT_MOTORS = 5;      // pin that controls the speed of the 2 left side motors - PWM pin
const int RIGHT_MOTORS = 6;     // pin that controls the speed of the 2 right side motors - PWM pin
const int LEFT_MOTOR_DIR = 8;   // pin that controls the polarity, or direction, of the 2 left side motors (HIGH = forward, LOW = backward)
const int RIGHT_MOTOR_DIR = 7;  // pin that controls the polarity, or direction, of the 2 right side motors (HIGH = forward, LOW = backward)
const int MOTOR_OUTPUT = 3;     // controls whether the motors are allowed to run or no

const int RIGHT = A0;
const int MIDDLE = A1;
const int LEFT = A2;

const int servoPin = 10;
const int triggerPin = 13;
const int echoPin = 12;
const int THRESHOLD_ABOVE_IS_BLACK = 450;
const int THRESHOLD_ABOVE_IS_NOISEOLD = 850;

NewPing sonar(triggerPin, echoPin, 30);
Servo servo;

void forward(int speed) {
  digitalWrite(MOTOR_OUTPUT, HIGH);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);

  analogWrite(LEFT_MOTORS, speed);
  analogWrite(RIGHT_MOTORS, speed);
}

void stop() {
  digitalWrite(MOTOR_OUTPUT, LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.write(155);
  pinMode(MOTOR_OUTPUT, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTORS, OUTPUT);
  pinMode(RIGHT_MOTORS, OUTPUT);
  pinMode(RIGHT, INPUT);
  pinMode(MIDDLE, INPUT);
  pinMode(LEFT, INPUT);
}

int blacklineCounter = 0;
void loop() {
  // put your main code here, to run repeatedly:
  int rightRead = analogRead(RIGHT);
  int middleRead = analogRead(MIDDLE);
  int leftRead = analogRead(LEFT);
  if (blacklineCounter < numLine) {
    int blackLine = 0;
    forward(40);
    while (
      rightRead > THRESHOLD_ABOVE_IS_BLACK && rightRead < THRESHOLD_ABOVE_IS_NOISEOLD || middleRead > THRESHOLD_ABOVE_IS_BLACK && middleRead < THRESHOLD_ABOVE_IS_NOISEOLD || leftRead > THRESHOLD_ABOVE_IS_BLACK && leftRead < THRESHOLD_ABOVE_IS_NOISEOLD) {

      blackLine = 1;

      rightRead = analogRead(RIGHT);
      middleRead = analogRead(MIDDLE);
      leftRead = analogRead(LEFT);
    }
    if (blackLine == 1) {
      blacklineCounter += 1;
      servo.write(125);
      delay(100);
      servo.write(155);
      delay(100);
    }
    if (blacklineCounter == numLine) {
      stop();
      delay(3000);
    }

  }

  else {
    forward(30);
    servo.write(125);
    delay(100);
    int cm = sonar.ping_cm();
    if (cm > 0 && cm <= distVal) {
      stop();
      while (1)
        ;
    }
  }
}