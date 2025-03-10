#include "goalie.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"
#include "vars.h"
#include <Arduino.h>

// TIMED PID CONTROL TESTING
void drivePID(signed int direzione, float vMot) {

  /*
    vx+ robot avanti
    vx- robot indietro
    vy+ robot sinistra
    vy- robot destra

    Il flip x-y è dovuto al passaggio dalla circonferenza goniometrica a qulella del robot, che ha lo 0 a 90° della circonferenza
    Il cambio di verso dell'asse y è dovuto al segno meno nella formula sotto
  */

  vx = ((vMot * cosin[direzione]));
  vy = ((-vMot * sins[direzione]));


  if((((vy < 0 && vxn == 1) || (vy > 0 && vxp == 1) || (vx < 0 && vyp == 1) || (vx > 0 && vyn == 1)) && canUnblock) || (millis() > unlockTime + UNLOCK_THRESH)) {
    vxn = 0;
    vxp = 0;
    vyp = 0;
    vyn = 0;
  }

  if((vy > 0 && vxn == 1) || (vy < 0 && vxp == 1)) vy = 0;
  if((vx > 0 && vyp == 1) || (vx < 0 && vyn == 1)) vx = 0; 
 

  speed1 = ((vx * sins[45] ) + (vy * cosin[45] ));
  speed2 = ((vx * sins[135]) + (vy * cosin[135]));
  speed3 = -(speed1);
  speed4 = -(speed2);

  pidfactor = updatePid();
  speed1 += pidfactor;
  speed2 += pidfactor;
  speed3 += pidfactor;
  speed4 += pidfactor;

  speed1 = constrain(speed1, -255, 255);
  speed2 = constrain(speed2, -255, 255);
  speed3 = constrain(speed3, -255, 255);
  speed4 = constrain(speed4, -255, 255);

  // speed1 = (int)speed1 > 0 ? map((int)speed1, 1, 255, 35, 255) : speed1;        //maggiore efficienza dei motori
  // speed2 = (int)speed2 > 0 ? map((int)speed2, 1, 255, 35, 255) : speed2;        //maggiore efficienza dei motori
  // speed3 = (int)speed3 > 0 ? map((int)speed3, 1, 255, 35, 255) : speed3;        //maggiore efficienza dei motori
  // speed4 = (int)speed4 > 0 ? map((int)speed4, 1, 255, 35, 255) : speed4;        //maggiore efficienza dei motori

  // speed1 = (int)speed1 < 0 ? map((int)speed1, -255, -1, -255, -35) : speed1;        //maggiore efficienza dei motori
  // speed2 = (int)speed2 < 0 ? map((int)speed2, -255, -1, -255, -35) : speed2;        //maggiore efficienza dei motori
  // speed3 = (int)speed3 < 0 ? map((int)speed3, -255, -1, -255, -35) : speed3;        //maggiore efficienza dei motori
  // speed4 = (int)speed4 < 0 ? map((int)speed4, -255, -1, -255, -35) : speed4;        //maggiore efficienza dei motori

  // Send every speed to his motor
  mot(1, int(speed1));
  mot(2, int(speed2));
  mot(3, int(speed3));
  mot(4, int(speed4));

  prevPidDir = direzione;
  prevPidSpeed = vMot;
}

float f = 0.6;

void preparePID(int direction, int speed) { preparePID(direction, speed, 0); }

void preparePID(int direction, int speed, int offset) {
  prevPidDir = globalDir;

  // globalDir = direction*f + prevPidDir* (1-f);
  globalDir = direction;
  globalSpeed = speed;
  st = offset;
  while(st < -180) st += 360;
  while(st > 180) st -= 360;
  // if(bounds) st = 0;
}


float updatePid() {
  float errorP, errorD, errorI;
  // assumiamo che la bussola dia un intero positivo, questo lo rende da -179 a +180
  float delta;  

  // calcola l'errore di posizione rispetto allo 0
  delta = imu_current_euler;
  if(delta > 180) delta = delta-360;
  delta = delta -st;

  // calcola correzione proporzionale
  errorP = KP * delta;

  // calcola correzione derivativa
  errorD = KD * (delta - errorePre);
  errorePre = delta;

  // calcola correzione integrativa
  integral = 0.5 * integral + delta;
  errorI = KI * integral;
  // calcota correzione complessiva
  return errorD + errorP + errorI;
}
