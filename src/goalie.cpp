#include "goalie.h"
#include "pid.h"
#include "position.h"
#include "us.h"
#include "vars.h"
#include "imu.h"
#include "camera.h" 
#include "config.h"
#include "music.h"
#include <Arduino.h>

int SCY1 = 0;
int SCY2 = 0;

void goalie() {
  digitalWrite(Y, LOW);
  x = 1;                                              
  y = 1;
  if(ball_degrees >= 350 || ball_degrees <= 10)  {
    if(ball_distance > 190) atk_direction = 0;
    else atk_direction = ball_degrees;
    atk_speed = GOALIE_ATKSPD_FRT;
  }

  if(ball_degrees >= 90 && ball_degrees <= 270) {
    ballBack();
    atk_speed = GOALIE_ATKSPD_BAK;                   
  }
  if(digitalRead(SWITCH_DX) == 1) {
    if(ball_degrees > 10   && ball_degrees <  30)   {
      atk_direction = ball_degrees + GOALIE_ATKDIR_PLUSANG1;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees >= 30 && ball_degrees <  45)    {
      atk_direction = ball_degrees + GOALIE_ATKDIR_PLUSANG2;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees >= 45 && ball_degrees <  90)    {
      atk_direction = ball_degrees + GOALIE_ATKDIR_PLUSANG3;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees > 270 && ball_degrees <= 315)   {
      atk_direction = ball_degrees - GOALIE_ATKDIR_PLUSANG3_COR;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees > 315 && ball_degrees <= 330)   {
      atk_direction = ball_degrees - GOALIE_ATKDIR_PLUSANG2_COR;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees > 330 && ball_degrees <  350)   {
      atk_direction = ball_degrees - GOALIE_ATKDIR_PLUSANG1_COR;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
  } else {
    if(ball_degrees > 10   && ball_degrees <  30)   {
      atk_direction = ball_degrees + GOALIE_ATKDIR_PLUSANG1_COR;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees >= 30 && ball_degrees <  45)    {
      atk_direction = ball_degrees + GOALIE_ATKDIR_PLUSANG2_COR;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees >= 45 && ball_degrees <  90)    {
      atk_direction = ball_degrees + GOALIE_ATKDIR_PLUSANG3_COR;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees > 270 && ball_degrees <= 315)   {
      atk_direction = ball_degrees - GOALIE_ATKDIR_PLUSANG3;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees > 315 && ball_degrees <= 330)   {
      atk_direction = ball_degrees - GOALIE_ATKDIR_PLUSANG2;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
    if(ball_degrees > 330 && ball_degrees <  350)   {
      atk_direction = ball_degrees - GOALIE_ATKDIR_PLUSANG1;
      atk_speed = GOALIE_ATKSPD_LAT;
    }
  }

  if((ball_degrees >= 330 || ball_degrees <= 30) && ball_distance > 190) {      //storcimento
    atk_speed = GOALIE_ATKSPD_STRK;                                             //dove i gigahertz hanno fallito
    preparePID(atk_direction, atk_speed, cstorc);
  }
  else  preparePID(atk_direction, atk_speed);
}

void leaveMeAlone() {
  if (zoneIndex >= 6) goCenter();
}

void storcimentoPorta() {
  if (fixCamIMU(pAtk) >= 3) cstorc+=9;
  else if (fixCamIMU(pAtk)  < -3) cstorc-=9;
  else cstorc *= 0.7;
  // else {
  //   if (cstorc > 0) cstorc -= 2;
  //   else  cstorc += 2;
  // }
  
  cstorc = constrain(cstorc, -30, 30);

  //SUPERTEAM
  // if (cstorc > 55) cstorc = 55;
  // if (cstorc < -55) cstorc = -55;
}

void ballBack() {
  int ball_degrees2;
  int dir;

  int plusang;
  if(ball_distance > 130) plusang = GOALIE_ATKDIR_PLUSANGBAK;
  else plusang = 0;
  
  if(ball_degrees > 180) ball_degrees2 = ball_degrees - 360;
  else ball_degrees2 = ball_degrees;
  if(ball_degrees2 > 0) dir = ball_degrees + plusang;              //45 con 8 ruote
  else dir = ball_degrees - plusang;                                 //45 con 8 ruote
  if(dir < 0) dir = dir + 360;
  else dir = dir;
  atk_direction = dir;
}
