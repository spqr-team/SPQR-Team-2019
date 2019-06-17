#include "goalie.h"
#include "pid.h"
#include "position.h"
#include "us.h"
#include "vars.h"
#include "imu.h"
#include "camera.h" 
#include "music.h"
#include <Arduino.h>

int SCY1 = 0;
int SCY2 = 0;
float cstorc = 0;

void goalie() {
  x = 1;
  y = 1;
  if(ball_degrees >= 345 || ball_degrees <= 15)  atk_direction = ball_degrees;
  if(ball_degrees > 15  && ball_degrees < 30)    atk_direction = ball_degrees + 20;
  if(ball_degrees >= 30 && ball_degrees < 45)    atk_direction = ball_degrees + 40;
  if(ball_degrees >= 45 && ball_degrees < 90)    atk_direction = ball_degrees + 50;
  if(ball_degrees >= 90 && ball_degrees < 145)   atk_direction = 170;
  if(ball_degrees >= 145&& ball_degrees <= 180)  atk_direction = 215;
  if(ball_degrees > 180 && ball_degrees <= 215)  atk_direction = 145;
  if(ball_degrees > 215 && ball_degrees <= 270)  atk_direction = 190;
  if(ball_degrees > 270 && ball_degrees <= 315)  atk_direction = ball_degrees - 50;
  if(ball_degrees > 315 && ball_degrees <= 330)  atk_direction = ball_degrees - 40;
  if(ball_degrees > 330 && ball_degrees < 345)   atk_direction = ball_degrees - 20;

  
  atk_speed = 255;

  goalPosition();
  if((ball_degrees >= 330 || ball_degrees <= 30) && ball_distance > 140) {
    if(cstorc > 20) {
      digitalWrite(R, HIGH);
      digitalWrite(Y, LOW);
    }
    if(cstorc < -20) {
      digitalWrite(R, LOW);
      digitalWrite(Y, HIGH);
    }
    preparePID(atk_direction, atk_speed, cstorc);
  }
  else preparePID(atk_direction, atk_speed);
}

void leaveMeAlone() {
  if (zoneIndex >= 6)
    goCenter();
}

void storcimentoPorta() {
  if (pAtk > 20) cstorc+=5;
  else if (pAtk < -20) cstorc-=5;
  else {
    if (cstorc > 0) cstorc -= 2;
    else  cstorc += 2;
  }
  
  
  if (cstorc > 30) cstorc = 30;
  if (cstorc < -30) cstorc = -30;
  //Serial.println(cstorc);
}
