#include "linesensor.h"
#include "goalie.h"
#include "position.h"
#include "pid.h"
#include "motors.h"
#include "music.h"
#include "vars.h"
#include "nano_ball.h"
#include "config.h"
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

bool fboundsX;
bool fboundsY;

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
    vyp = 0;
    vyn = 0;
    vxp = 0;
    vxn = 0;
    if(exitTimer > EXTIME) {
      fboundsX = true;
      fboundsY = true;
    }
    exitTimer = 0;
  }

  linesensbyte |= (linesensbyteI | linesensbyteO);

  outOfBounds();
}

void outOfBounds(){

  if(fboundsX == true) {
    if(linesensbyte & 0x02) linesensbyteOLDX = 2;
    else if(linesensbyte & 0x08) linesensbyteOLDX = 8;
    if(linesensbyteOLDX != 0) fboundsX = false;
  }
  if(fboundsY == true) {
    if(linesensbyte & 0x01) linesensbyteOLDY = 1;
    else if(linesensbyte & 0x04) linesensbyteOLDY = 4;
    if(linesensbyteOLDY != 0) fboundsY = false;
  }

  if (exitTimer <= EXTIME){
    //fase di rientro
    if(linesensbyte == 15) linesensbyte = linesensbyteOLDY | linesensbyteOLDX;        //ZOZZATA MAXIMA
    unlockTime = millis();

    if(linesensbyte == 1) outDir = 180;
    else if(linesensbyte == 2) outDir = 270;
    else if(linesensbyte == 4) outDir = 0;
    else if(linesensbyte == 8) outDir = 90;
    else if(linesensbyte == 3) outDir = 225;
    else if(linesensbyte == 6) outDir = 315;
    else if(linesensbyte == 12) outDir = 45;
    else if(linesensbyte == 9) outDir = 135;
    else if(linesensbyte == 7) outDir = 270;
    else if(linesensbyte == 13) outDir = 90;
    else if(linesensbyte == 11) outDir = 180;
    else if(linesensbyte == 14) outDir = 0;
    else if(linesensbyte == 5){
      if(linesensbyteOLDX == 2) outDir = 270;
      else if(linesensbyteOLDY == 8) outDir = 90;
    }
    else if(linesensbyte == 10){
      if(linesensbyteOLDY == 4) outDir = 0;
      else if(linesensbyteOLDY == 1) outDir = 180;
    }

    outVel = LINES_EXIT_SPD;
    preparePID(outDir, outVel, 0);
    
    keeper_backToGoalPost = true;
    keeper_tookTimer = true;
  }else{
    //fine rientro
    if(linesensbyte == 1) vyp = 1;
    else if(linesensbyte == 2) vxp = 1;
    else if(linesensbyte == 4) vyn = 1;
    else if(linesensbyte == 8) vxn = 1;
    else if(linesensbyte == 3) {
      vyp = 1;
      vxp = 1;
    }
    else if(linesensbyte == 6){
      vxp = 1;
      vyn = 1;
    }
    else if(linesensbyte == 12) {
      vyn = 1;
      vxn = 1;
    }
    else if(linesensbyte == 9) {
      vyp = 1;
      vxn = 1;
    }
    else if(linesensbyte == 7) {
      vyp = 1;
      vyn = 1;
      vxp = 1;
    }
    else if(linesensbyte == 13){
      vxp = 1;
      vxn = 1;
      vyn = 1;
    }
    else if(linesensbyte == 11) {
      vyp = 1;
      vxn = 1;
      vxp = 1;
    }
    else if(linesensbyte == 14) {
      vyn = 1;
      vxn = 1;
      vxp = 1;
    }
    else if(linesensbyte == 5){
      if(linesensbyteOLDX == 2) vyp = 1;
      else if(linesensbyteOLDY == 8)vyn = 1;
    }
    else if(linesensbyte == 10){
      if(linesensbyteOLDY == 4) vyn = 1;
      else if(linesensbyteOLDY == 1) vyp = 1;
    }

    linesensbyte = 0;
    linesensbyteOLDY = 0;
    linesensbyteOLDX = 0;
    lineSensByteBak = 30;

    canUnblock = true;
  }
   lineSensByteBak = linesensbyte;
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
  DEBUG_PRINT.println("________________");

  DEBUG_PRINT.println("Byte:    ");
  DEBUG_PRINT.print(linesensbyteO);
  DEBUG_PRINT.print("   Direzione:    ");
  DEBUG_PRINT.println(outDir);
  delay(150);

  DEBUG_PRINT.println("______________");
}