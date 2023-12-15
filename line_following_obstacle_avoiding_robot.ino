#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

//hc-sr04 sensor
#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define max_distance 50

//ir sensor
#define irFrontLeft A0
#define irFrontRight A1
#define irLeft 7
#define irRight 8

//motor
#define MAX_SPEED 150
#define MAX_SPEED_OFFSET 20

Servo servo;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

int table = 1;
int counter = 0;

int distance = 0;
int leftDistance;
int rightDistance;
boolean object;
bool ignoreHaltingPosition = false;

void setup() {
  AFMS.begin();
  Serial.begin(9600);
  pinMode(irFrontLeft, INPUT);
  pinMode(irFrontRight, INPUT);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(10);
  servo.write(90);

  motor1->setSpeed(180);
  motor2->setSpeed(180);
  motor3->setSpeed(180);
  motor4->setSpeed(180);
}

void loop() {
  distance = getDistance();

  if (distance <= 15){
    Stop();
    loop();
  }else{
    if (irLeft == 1 && irRight == 1) {
       Serial.println("irLeft1");
       Serial.println(irLeft);
       Serial.println("irRight");
       Serial.println(irRight);
      if (table == 1){
        Stop();
        delay(2000);
        loop();
      }
    }
    else if (irLeft == 1 && irRight == 1 && (irFrontLeft == 1 || irFrontRight == 1)) {
      if (table == 2){
        Stop();
        delay(2000);
        loop();
      }
    }
    else {
      // Follow the line if there is not stop and no obstacle
       if (digitalRead(irFrontLeft) == 0 && digitalRead(irFrontRight) == 0) {
        moveForward();
      } else if (digitalRead(irFrontLeft) == 1 && digitalRead(irFrontRight) == 0) {
        moveLeft();
      } else if (digitalRead(irFrontLeft) == 0 && digitalRead(irFrontRight) == 1) {
        moveRight();
      } else {
        Stop();
      }
    }
    delay(100); // Small delay for stability
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

void Stop() {
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
}

void moveForward() {
  motor1->setSpeed(120);
  motor2->setSpeed(120);
  motor3->setSpeed(120);
  motor4->setSpeed(120);

  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor3->run(FORWARD);
  motor4->run(FORWARD);
}

void moveBackward() {
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor3->run(BACKWARD);
  motor4->run(BACKWARD);
}

void turn() {
  if (object == false) {//object at left
    Serial.println("turn Right");
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(900);
    if (digitalRead(irFrontRight) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
  else {//object at right
    Serial.println("turn left");
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(900);
    if (digitalRead(irFrontLeft) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
}
void moveRight() {
  motor1->setSpeed(120);
  motor2->setSpeed(120);
  motor3->setSpeed(120);
  motor4->setSpeed(120);

  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor3->run(FORWARD);
  motor4->run(FORWARD);
}

void moveLeft() {
  motor1->setSpeed(120);
  motor2->setSpeed(120);
  motor3->setSpeed(120);
  motor4->setSpeed(120);

  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor3->run(BACKWARD);
  motor4->run(BACKWARD);
}