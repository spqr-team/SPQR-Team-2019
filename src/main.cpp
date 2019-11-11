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

  // Now assign value to variables, first thing to do
  // IMU
  imu_current_euler = 0;
  // Ball
  ball_distance = 0;
  ball_degrees = 0;
  ball_seen = false;
  // PID
  errorePre = 0.0;
  integral = 0.0;
  st = 0;
  // US
  reading = 0;
  us_t0 = 0;
  us_t1 = 0;
  us_flag = false;
  // Position
  old_status_x = CENTER;
  old_status_y = CENTER;
  // old_guessedlocation = CENTER_CENTER;
  goal_zone = false;
  good_field_x = true;
  good_field_y = true;
  status_x = CENTER;
  status_y = CENTER;
  // currentlocation = CENTER_CENTER;
  // guessedlocation = CENTER_CENTER;
  // Linesensors and interrupt

  // bluetooth misc
  a = 0;
  old_timer = 0;
  role = 0;
  friendZone = 0;
  iAmHere = 0;
  comrade = false;

  // global vars for angles
  globalDir = 0;
  globalSpeed = 0;
  st = 0;

  // attack
  atk_direction = 0;
  atk_speed = 0;

  // CAMERA
  pAtk = 0;
  pDef = 0;
  portx = 0;
  goal_orientation = 0;
  cameraReady = 0;

  // BT
  topolino = 0;
  fpos = 0;

  // stincr
  stincr = 0;
  cstorc = 0;

  // lines
  exitTimer = EXTIME;

  //axis
  y = 1;
  x = 1;

  keeper_tookTimer = false;
  keeper_backToGoalPost = false;

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
  //Enable Serial3 for debug

  // Enable Serial4 for the slave
  NANO_BALL.begin(57600);
  // Enable Serial2 for the camera
  CAMERA.begin(19200);

  // Misc inits
  initIMU();
  initMotorsGPIO();
  initUS();
  initSinCos();



  timertest = 0;

  // if(digitalRead(SWITCH_DX) == HIGH && digitalRead(SWITCH_SX) == HIGH){
  //   NANO_BALL.end();
    // //pinMode(31, INPUT_DISABLE);
    // //pinMode(32, INPUT_DISABLE);
    // tone(BUZZER, C6);
    // delay(500);
    // noTone(BUZZER);
    // super_mario();
  // }
  
  delay(400);

  initBluetooth();

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
  
  // Ao();
  // com(500);

  // comrade = true;

  if(comrade) {
    if(ball_seen){
      if(role) goalie();
      else keeper();
    } else {
      if(role){
        // goCenter();
        preparePID(0,0,0);
        digitalWrite(Y, LOW);
      }
      else centerGoalPostCamera(true);
    }
  }else{
    if(ball_seen) keeper();
    else centerGoalPostCamera(true);
  }

  AAANGOLO(); 

  checkLineSensors();                           //Last thing in loop, for priority
  // safetysafe();

  drivePID(globalDir, globalSpeed);
  // for(int i = 0; i <= 255; i+= 10) {
  //   turnMotor(2, 1, 0, 50);
  //   if(i >= 255) i = 0;
  //   Serial.print(i);
  //   Serial.print(", ");
  //   while(!Serial.available());
  //   Serial.println(Serial.parseFloat(),4);
  //   while(Serial.available()) Serial.read();
  // } 
  
}
