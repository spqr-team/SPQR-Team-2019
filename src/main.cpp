#include "libs.h"
#include "imu.h"
#include "us.h"
#include "motors.h"
#include "spi.h"
#include "linesensor.h"


//Switch management vars
int SWS = 0;
int SWD = 0;


void setup() {
  Serial.begin(9600);
  pinMode(LN1, INPUT);
  attachInterrupt(LN1, testInterrupt, FALLING);
  //Wire1.begin();
  //initIMU();
  //initSPI();

  //init_linesensors();
  //SWS = digitalRead(SWITCH_SX);
  /*valStringY.rese
  rve(30);                                     //riserva 30byte per le stringhe
  valStringB.reserve(30);*/

  //initMotorsGPIO();
  //tone(22, 1000, 500);
  //for(int i=29;i<=31;i++) pinMode(i, OUTPUT);

}

void loop() {

  //readSPI();

  //Serial.println("ao");
  //Serial.println(digitalRead(LN1));

  delay(500);
  //readIMU();
  //readUS();

  /*
  turnMotor(1,0,1,255);
  delay(500);
  turnMotor(1,0,0,0);u
  delay(1000);
  turnMotor(1,1,0,255);
  delay(500);
  turnMotor(1,0,0,0);
  delay(1000);
  */

}
