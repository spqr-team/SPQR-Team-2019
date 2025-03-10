#include "motors.h"
#include "vars.h"
#include "pid.h"
#include <Arduino.h>

byte INA_MOT[5] = {0, 12, 25, 27, 21}; //{0,  16,  5,  8};           // INA pin
byte INB_MOT[5] = {0, 11, 24, 26, 22}; //{0,  15,  6,  9};           // INB pin
byte PWM_MOT[5] = {0, 2,   5,  6, 23}; //{0,  4,   7,   10};         // PWM pin

void initMotorsGPIO() {
  pinMode(PWM_MOT[1], OUTPUT);
  pinMode(INA_MOT[1], OUTPUT);
  pinMode(INB_MOT[1], OUTPUT);
  pinMode(PWM_MOT[2], OUTPUT);
  pinMode(INA_MOT[2], OUTPUT);
  pinMode(INB_MOT[2], OUTPUT);
  pinMode(PWM_MOT[3], OUTPUT);
  pinMode(INA_MOT[3], OUTPUT);
  pinMode(INB_MOT[3], OUTPUT);
  pinMode(PWM_MOT[4], OUTPUT);
  pinMode(INA_MOT[4], OUTPUT);
  pinMode(INB_MOT[4], OUTPUT);
}

/**
 * Combination of motor 1 / motor 2 /result
 *
 * 0 - 0   ----->  Stopped - Neutral
 * 0 - 1   ----->  Clockwise
 * 1 - 0   ----->  Counter-Clockwise
 * 1 - 1   ----->  Stopped - Brake
 *
 **/

void turnMotor(byte motorIndex, byte pinA, byte pinB, byte pwm) {
  digitalWrite(INA_MOT[motorIndex], pinA);
  digitalWrite(INB_MOT[motorIndex], pinB);
  analogWrite(PWM_MOT[motorIndex], pwm);
}

void brake() {
  digitalWrite(INA_MOT[1], 1);
  digitalWrite(INB_MOT[1], 1);
  digitalWrite(INA_MOT[2], 1);
  digitalWrite(INB_MOT[2], 1);
  digitalWrite(INA_MOT[3], 1);
  digitalWrite(INB_MOT[3], 1);
  digitalWrite(INA_MOT[4], 1);
  digitalWrite(INB_MOT[4], 1);
  analogWrite(PWM_MOT[1], 255);
  analogWrite(PWM_MOT[2], 255);
  analogWrite(PWM_MOT[3], 255);
  analogWrite(PWM_MOT[4], 255);
  return;
}


void brakeI() {
  digitalWrite(INA_MOT[1], 1);
  digitalWrite(INB_MOT[1], 1);
  digitalWrite(INA_MOT[2], 1);
  digitalWrite(INB_MOT[2], 1);
  digitalWrite(INA_MOT[3], 1);
  digitalWrite(INB_MOT[3], 1);
  digitalWrite(INA_MOT[4], 1);
  digitalWrite(INB_MOT[4], 1);
  analogWrite(PWM_MOT[1], 255);
  analogWrite(PWM_MOT[2], 255);
  analogWrite(PWM_MOT[3], 255);
  analogWrite(PWM_MOT[4], 255);
  return;
}

// degrees to radiant converting
float torad(float deg) {
  return (deg * PI / 180.0);
}
// calculates sins of integer angles from 0 to 359
void initSinCos() { 
  for (int i = 0; i < 360; i++) {
    sins[i] = sin(torad(i));
  }
  for (int i = 0; i < 360; i++) {
    cosin[i] = cos(torad(i));
  }
}

// Function to send the speed to the motor
void mot(byte mot, int vel) {
  byte VAL_INA, VAL_INB;
  if (vel == 0) {
    // no brake ma motore inerte corto a massa e vel=0 contro freno
    // dove corto a VCC e vel=max
    VAL_INA = 0;
    VAL_INB = 0;
  } else if (vel > 0) {
    // clockwise
    VAL_INA = 1;
    VAL_INB = 0;
  } else if (vel < 0) {
    // counterclockwise
    VAL_INA = 0;
    VAL_INB = 1;
    vel = -vel;
  }
  digitalWrite(INA_MOT[mot], VAL_INA);
  digitalWrite(INB_MOT[mot], VAL_INB);
  analogWrite(PWM_MOT[mot], vel);
  return;
}

void testMotors() {
  for (int i = 1; i < 5; i++) {
    turnMotor(i, 0, 1, 100);
    delay(1000);
    turnMotor(i, 0, 0, 100);
    delay(300);
    turnMotor(i, 1, 0, 100);
    delay(1000);
    turnMotor(i, 0, 0, 100);
    delay(300);
  }
    // turnMotor(1, 0, 1, 80);
    // turnMotor(3, 0, 1, 200);
    // delay(1000);
    // turnMotor(1, 0, 0, 80);
    // turnMotor(3, 0, 0, 200);
    // delay(300);
    // turnMotor(1, 1, 0, 80);
    // turnMotor(3, 1, 0, 200);
    // delay(1000);
    // turnMotor(1, 0, 0, 80);
    // turnMotor(3, 0, 0, 200);
    // delay(300);

    // turnMotor(2, 0, 1, 80);
    // turnMotor(4, 0, 1, 200);
    // delay(1000);
    // turnMotor(2, 0, 0, 80);
    // turnMotor(4, 0, 0, 200);
    // delay(300);
    // turnMotor(2, 1, 0, 80);
    // turnMotor(4, 1, 0, 200);
    // delay(1000);
    // turnMotor(2, 0, 0, 80);
    // turnMotor(4, 0, 0, 200);
    // delay(300);

    // turnMotor(3, 0, 1, 80);
    // turnMotor(1, 0, 1, 200);
    // delay(1000);
    // turnMotor(3, 0, 0, 80);
    // turnMotor(1, 0, 0, 200);
    // delay(300);
    // turnMotor(3, 1, 0, 80);
    // turnMotor(1, 1, 0, 200);
    // delay(1000);
    // turnMotor(3, 0, 0, 80);
    // turnMotor(1, 0, 0, 200);
    // delay(300);

    // turnMotor(4, 0, 1, 80);
    // turnMotor(2, 0, 1, 200);
    // delay(1000);
    // turnMotor(4, 0, 0, 80);
    // turnMotor(2, 0, 0, 200);
    // delay(300);
    // turnMotor(4, 1, 0, 80);
    // turnMotor(2, 1, 0, 200);
    // delay(1000);
    // turnMotor(4, 0, 0, 80);
    // turnMotor(2, 0, 0, 200);
    // delay(300);
}