#include "goalie.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"
#include "vars.h"
#include <Arduino.h>

// TIMED PID CONTROL TESTING
void drivePID(signed int direzione, float vMot) {
  vx = ((vMot * cosin[direzione]) * x);
  vy = ((-vMot * sins[direzione]) * y);

  speed1 = ((vx * sins[45] ) + (vy * cosin[45] ));
  speed2 = ((vx * sins[135]) + (vy * cosin[135]));
  speed3 = -(speed1);
  speed4 = -(speed2);

  /*speed1 = ((-(sins[((direzione - 45) + 360) % 360])) * vMot);          // mot 1
   speed2 = ((-(sins[((direzione - 135) + 360)% 360])) * vMot);          // mot 2
   speed3 = -(speed1);                                                   // mot 3
   speed4 = -(speed2);*/                                                   // mot 4

  pidfactor = updatePid();


  speed1 += pidfactor;
  speed2 += pidfactor;
  speed3 += pidfactor;
  speed4 += pidfactor;

  // int pidfactor2 = -pAtk * 2.5;
  //if((ball_degrees >= 330 || ball_degrees <= 30) && ball_distance > 140) pidfactor2 = 0;

  // speed1 += pidfactor2;
  // speed2 += pidfactor2;
  // speed3 += pidfactor2;
  // speed4 += pidfactor2;

  speed1 = constrain(speed1, -255, 255);
  speed2 = constrain(speed2, -255, 255);
  speed3 = constrain(speed3, -255, 255);
  speed4 = constrain(speed4, -255, 255);

  speed1 = (int)speed1 > 0 ? map((int)speed1, 1, 255, 35, 255) : speed1;        //maggiore efficienza dei motori
  speed2 = (int)speed2 > 0 ? map((int)speed2, 1, 255, 35, 255) : speed2;        //maggiore efficienza dei motori
  speed3 = (int)speed3 > 0 ? map((int)speed3, 1, 255, 35, 255) : speed3;        //maggiore efficienza dei motori
  speed4 = (int)speed4 > 0 ? map((int)speed4, 1, 255, 35, 255) : speed4;        //maggiore efficienza dei motori

  speed1 = (int)speed1 < 0 ? map((int)speed1, -255, -1, -255, -35) : speed1;        //maggiore efficienza dei motori
  speed2 = (int)speed2 < 0 ? map((int)speed2, -255, -1, -255, -35) : speed2;        //maggiore efficienza dei motori
  speed3 = (int)speed3 < 0 ? map((int)speed3, -255, -1, -255, -35) : speed3;        //maggiore efficienza dei motori
  speed4 = (int)speed4 < 0 ? map((int)speed4, -255, -1, -255, -35) : speed4;        //maggiore efficienza dei motori



  // Send every speed to his motor
  mot(1, int(speed1));
  mot(2, int(speed2));
  mot(3, int(speed3));
  mot(4, int(speed4));

  prevPidDir = direzione;
  prevPidSpeed = vMot;
}

void preparePID(int direction, int speed) { preparePID(direction, speed, 0); }

void preparePID(int direction, int speed, int offset) {
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
