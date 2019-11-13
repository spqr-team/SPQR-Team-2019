#include "vars.h"

void initVars(){


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
  old_status_x = CENTRO;
  old_status_y = CENTRO;
  // old_guessedlocation = CENTER_CENTER;
  goal_zone = false;
  good_field_x = true;
  good_field_y = true;
  status_x = CENTRO;
  status_y = CENTRO;
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

  keeper_tookTimer = false;
  keeper_backToGoalPost = false;


  //axis
  y = 1;
  x = 1;
}

