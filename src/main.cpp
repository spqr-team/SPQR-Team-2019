// Here to correctly init vars.h, do not remove
#define MAIN

#include "Wire.h"
#include <Arduino.h>

#include "bluetooth.h"
#include "camera.h"
#include "chat.h"
#include "goalie.h"
#include "imu.h"
#include "linesensor.h"
#include "motors.h"
#include "music.h"
#include "nano_ball.h"
#include "pid.h"
#include "position.h"
#include "test.h"
#include "keeper.h"
#include "us.h"
#include "vars.h"
#include "config.h"

// Switch management vars
int SWS = 0;
int SWD = 0;
elapsedMillis timertest;
int aiut = 0;

void setup() {
  startSetup();
  initVars();

  // ;)
  analogWriteFrequency(2 , 15000);
  analogWriteFrequency(5 , 15000);
  analogWriteFrequency(6,  15000);
  analogWriteFrequency(23, 15000);

  // disable those pins, damaged teensy
  pinMode(A8, INPUT_DISABLE); // pin A8 in corto tra 3.3V e massa
  pinMode(16, INPUT_DISABLE); // pin 16 in corto tra 3.3V e massa
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(Y, OUTPUT);
  pinMode(SWITCH_DX, INPUT);
  pinMode(SWITCH_SX, INPUT);

  // Enable Serial for test
  Serial.begin(9600);
  // Enable Serial4 for the slave
  NANO_BALL.begin(57600);
  // Enable Serial2 for the camera
  CAMERA.begin(19200);

  // Misc inits
  initIMU();
  initMotorsGPIO();
  initUS();
  initSinCos();

  initBluetooth();

  timertest = 0;
  delay(400);
  stopSetup();
}

void loop() {
  role = digitalRead(SWITCH_DX);                //se HIGH sono attaccante
  goal_orientation = digitalRead(SWITCH_SX);     //se HIGH attacco gialla, difendo blu

  if(DEBUG_PRINT.available() > 0) testMenu();

  readIMU();
  readUS();
  readBallNano();
  goalPosition();

  if(cameraReady == 1) {
    storcimentoPorta();
    calcPhyZoneCam = true;
    cameraReady = 0;
  }

  calculateLogicZone();
  comrade = true;
  
  if(comrade) {
    if(ball_seen){
      if(role) goalie();
      else keeper();
    } else {
      if(role) preparePID(0,0,0);
      else centerGoalPostCamera(true);
    }
  }else{
    if(ball_seen) keeper();
    else centerGoalPostCamera(true);
  }

  //Modify the speed BEFORE the line sensors checking (lines need to go to max speed possible)
  // globalSpeed *= GLOBAL_SPD_MULT;

  globalSpeed *= GLOBAL_SPD_MULT;
  // AAANGOLO();

  checkLineSensors();                           //Last thing in loop, for priority

  drivePID(globalDir, globalSpeed);

}

//SQUARE TO TEST MOVEMENTS
// if(timertest >= 0 && timertest < 1000) drivePID(0, 250);
// else if(timertest >= 1000 && timertest < 2000) drivePID(90, 250);
// else if(timertest >= 2000 && timertest < 3000) drivePID(180, 250);
// else if(timertest >= 3000 && timertest < 4000) drivePID(270, 250);
// else timertest = 0;

