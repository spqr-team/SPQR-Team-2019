#include "linesensor.h"
#include "goalie.h"
#include "position.h"
#include "pid.h"
#include "motors.h"
#include "music.h"
#include "vars.h"
#include "nano_ball.h"
#include <Arduino.h>

int linepinsI[4] = {S1I, S2I, S3I, S4I};
int linepinsO[4] = {S1O, S2O, S3O, S4O};
int linetriggerI[4];
int linetriggerO[4];
int lineCnt;
int CNTY;
int CNTX;
int prevDir;
int ai, ar;

byte linesensbyteI;
byte linesensbyteO;
byte linesensbyte;
byte linesensbytefst;
byte linesensbyteOLDY;
byte linesensbyteOLDX;
byte linesensbyteOfst;
byte linesensbyteOOLDY;
byte linesensbyteOOLDX;

bool fboundsX;
bool fboundsY;

bool fboundsOX;
bool fboundsOY;

int outDir;
int outVel;

elapsedMillis ltimer;

void checkLineSensors() {
  linesensbyteI = 0;
  linesensbyteO = 0;

  for(int i = 0; i < 4; i++) {
    linetriggerI[i] = analogRead(linepinsI[i]) > LINE_THRESH;
    linetriggerO[i] = analogRead(linepinsO[i]) > LINE_THRESH;
    linesensbyteI = linesensbyteI | (linetriggerI[i]<<i);
    linesensbyteO = linesensbyteO | (linetriggerO[i]<<i);
  }

  if ((linesensbyteI > 0) || (linesensbyteO > 0)) {
      fboundsOX = true;
      fboundsOY = true;
    if(exitTimer > EXTIME) {
      fboundsX = true;
      fboundsY = true;

    }
    exitTimer = 0;
  }

  // linesensbyte |= (linesensbyteI | linesensbyteO);
  linesensbyteI |= (linesensbyteI | linesensbyteO);

  outOfBounds();
}

void outOfBounds(){
    handleExtern();
    handleIntern();
  }

void handleExtern (){
    if((linesensbyteI & 0b00000001) == 1) vyp = 1;  // esclusione
    if((linesensbyteI & 0b00000100) == 4) vyn = 1;
    if((linesensbyteI & 0b00000010) == 2) vxp = 1;
    if((linesensbyteI & 0b00001000) == 8) vxn = 1;
}

void handleIntern(){
  if(fboundsX == true) {
    if(linesensbyteI & 0x02) linesensbyteOLDX = 2;
    else if(linesensbyteI & 0x08) linesensbyteOLDX = 8;
    if(linesensbyteOLDX != 0) fboundsX = false;
  }
  if(fboundsY == true) {
    if(linesensbyteI & 0x01) linesensbyteOLDY = 1;
    else if(linesensbyteI & 0x04) linesensbyteOLDY = 4;
    if(linesensbyteOLDY != 0) fboundsY = false;
  }
  if (exitTimer <= EXTIME){
    canUnblock = false;
    unlockTime = millis();

    //fase di rientro
    if(linesensbyteI == 15) {
      linesensbyteI = linesensbyteOLDY | linesensbyteOLDX;        //ZOZZATA MAXIMA
      //digitalWrite(Y, HIGH);
    }

    switch(linesensbyteI){
      case 1:
        outDir = 180;
        break;
      case 2:
        outDir = 270;
        break;
      case 4:
        outDir = 0;
        break;
      case 8:
        outDir = 90;
        break;
      case 3:
        outDir = 225;
        break;
      case 6:
        outDir = 315;
        break;
      case 12:
        outDir = 45;
        break;
      case 9:
        outDir = 135;
        break;
      case 7:
        outDir = 270;
        break;
      case 13:
        outDir = 90;
        break;
      case 11:
        outDir = 180;
        break;
      case 14:
        outDir = 0;
        break;
      case 5:
        //digitalWrite(R, HIGH);
        if(linesensbyteOLDX == 2) outDir = 270;
        if(linesensbyteOLDX == 8) outDir = 90;
        break;
      case 10:
        if(linesensbyteOLDY == 4) outDir = 0;
        if(linesensbyteOLDY == 1)outDir = 180;
        break;
      case 15:
        break;
      case 0:
      default:
        //;)
        break;
    }

    if(exitTimer < 45) outVel = 350;
    else outVel = 320;
    preparePID(outDir, outVel, 0);
    
    // tone(30, LA3);
    keeper_backToGoalPost = true;
    keeper_tookTimer = true;
  }else{
    linesensbyteI = 0;
    linesensbyteOLDY = 0;
    linesensbyteOLDX = 0;
    lineSensByteBak = 30;
    canUnblock = true;
  }

   lineSensByteBak = linesensbyteI;
   if(exitTimer == 99) slow = true;
   else slow = false;
}

void testLineSensors() {
  checkLineSensors();
  DEBUG_PRINT.println("______________");
  for(int i = 0; i<4; i++) {
    DEBUG_PRINT.print(linetriggerI[i]);
    DEBUG_PRINT.print("-");
    DEBUG_PRINT.print(linetriggerO[i]);
    DEBUG_PRINT.print(" | ");
  }
  DEBUG_PRINT.println();
  DEBUG_PRINT.print("________________");

  DEBUG_PRINT.println("Byte:    ");
  DEBUG_PRINT.print(linesensbyteO);
  DEBUG_PRINT.print("   Direzione:    ");
  DEBUG_PRINT.println(outDir);
  delay(150);

  DEBUG_PRINT.println("______________");
}