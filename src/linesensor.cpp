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
    if(exitTimer > EXTIME) {
      fboundsX = true;
      fboundsY = true;

      fboundsOX = true;
      fboundsOY = true;
    }
    exitTimer = 0;
  }

  // linesensbyte |= (linesensbyteI | linesensbyteO);

  outOfBounds();
}

void outOfBounds(){
  if(linesensbyteO > 0) handleExtern();
  else{
      linesensbyteOOLDX = 0;
      linesensbyteOOLDY = 0;
      vxp = 0;
      vxn = 0;
      vyp = 0;
      vyn = 0;  
  }
  if(linesensbyteI > 0) handleIntern();
}

void handleExtern (){

  if(fboundsOX == true) {
    if(linesensbyteO & 0x02) linesensbyteOOLDX = 2;
    else if(linesensbyteO & 0x08) linesensbyteOOLDX = 8;
    if(linesensbyteOOLDX != 0) fboundsOX = false;
  }
  if(fboundsOY == true) {
    if(linesensbyteO & 0x01) linesensbyteOOLDY = 1;
    else if(linesensbyteO & 0x04) linesensbyteOOLDY = 4;
    if(linesensbyteOOLDY != 0) fboundsOY = false;
  }
  if (exitTimer <= EXTIME){
    if((linesensbyteOOLDY & 0b00000001) == 1) vyp = 1;  // esclusione
    if((linesensbyteOOLDY & 0b00000100) == 4) vyn = 1;
    if((linesensbyteOOLDX & 0b00000010) == 2) vxp = 1;
    if((linesensbyteOOLDX & 0b00001000) == 8) vxn = 1;
  }
  DEBUG_PRINT.print(linesensbyteOOLDX);
  DEBUG_PRINT.print(" | ");
  DEBUG_PRINT.print(linesensbyteOOLDY);
  DEBUG_PRINT.print(" |     ");
  DEBUG_PRINT.print(fboundsOX);
  DEBUG_PRINT.print(" | ");
  DEBUG_PRINT.print(fboundsOY);
  DEBUG_PRINT.print(" |    ");
  DEBUG_PRINT.print(vxp);
  DEBUG_PRINT.print(" | ");
  DEBUG_PRINT.print(vxn);
  DEBUG_PRINT.print(" | ");
  DEBUG_PRINT.print(vyp);
  DEBUG_PRINT.print(" | ");
  DEBUG_PRINT.print(vyn);
  DEBUG_PRINT.println(" | ");
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
    //fase di rientro
    if(linesensbyteI == 15) {
      linesensbyteI = linesensbyteOLDY | linesensbyteOLDX;        //ZOZZATA MAXIMA
      //digitalWrite(Y, HIGH);
    }

    switch(linesensbyteI){
      case 1:
        outDir = 180;
        outVel = 250;
        ai = 0;
        ar = 30;
        break;
      case 2:
        outDir = 270;
        outVel = 250;
        ai = 90;
        ar = 60;
        break;
      case 4:
        outDir = 0;
        outVel = 250;
        ai = 180;
        ar = 30;
        break;
      case 8:
        outDir = 90;
        outVel = 250;
        ai = 270;
        ar = 60;
        break;
      case 3:
        outDir = 225;
        outVel = 250;
        ai = 45;
        ar = 20;
        break;
      case 6:
        outDir = 315;
        outVel = 250;
        ai = 135;
        ar = 20;
        break;
      case 12:
        outDir = 45;
        outVel = 250;
        ai = 225;
        ar = 20;
        break;
      case 9:
        outDir = 135;
        outVel = 250;
        ai = 315;
        ar = 20;
        break;
      case 7:
        outDir = 270;
        outVel = 250;
        ai = 90;
        ar = 60;
        break;
      case 13:
        outDir = 90;
        outVel = 250;
        ai = 270;
        ar = 60;
        break;
      case 11:
        outDir = 180;
        outVel = 250;
        ai = 0;
        ar = 60;
        break;
      case 14:
        outDir = 0;
        outVel = 250;
        ai = 180;
        ar = 60;
        break;


      case 5:
        //digitalWrite(R, HIGH);
        if(linesensbyteOLDX == 2) {
          outDir = 270;
          ai = 90;
          ar = 30;
        }
        if(linesensbyteOLDX == 8) {
          outDir = 90;
          ai = 270;
          ar = 30;
        }
        outVel = 250;
        break;
      case 10:
        //digitalWrite(G, HIGH);
        if(linesensbyteOLDY == 4) {
          outDir = 0;
          ai = 180;
          ar = 30;
        }
        if(linesensbyteOLDY == 1) {
          outDir = 180;
          ai = 0;
          ar = 30;
        }
        outVel = 250;
        break;
      case 15:

        break;
      case 0:
      default:
        //;)
        break;
    }
    // ballMask(1);
    if(exitTimer < 45) outVel = 330;
    else outVel = 230;
    preparePID(outDir, outVel, 0);
    
    // tone(30, LA3);
    keeper_backToGoalPost = true;
    keeper_tookTimer = true;
  }else{
    //fine rientro
    // ballMask(0);
    vxp = 0;
    vxn = 0;
    vyp = 0;
    vyn = 0;

    linesensbyteI = 0;
    linesensbyteOLDY = 0;
    linesensbyteOLDX = 0;
    // noTone(30);
    //digitalWrite(Y, LOW);
    //digitalWrite(G, LOW);

    lineSensByteBak = 30;
  }

   lineSensByteBak = linesensbyteI;
   if(exitTimer == 99) slow = true;
   else slow = false;
}

// int ball = -1;
// elapsedMillis mask;
// elapsedMillis slowly;

// void ballMask(int on) {
//   float diffB;

//   if(on) {
//     ball = ball_degrees;
//     mask = 0;
//   }
//   else {
//     if(ball != -1) {
//       if(inAngle(ball_degrees, ai, ar) == 0) {
//         ball = -1;
//         return;
//       }
//       else {
//         if(mask < 500) preparePID(0, 0, 0);         //prima era 150
//         else {
//           ball = -1;
//           return;
//         }
//       }
//     } else return;
//   }

// }

// void safetysafe() {
//   if(slow)  slowly = 0;
//   if(!slow) if(slowly < 600){
//     if(ball_degrees > 45 && ball_degrees < 315) globalSpeed = globalSpeed / 1.4;
//   }
// }


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