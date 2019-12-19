#include "pid.h"
#include "us.h"
#include "camera.h"
#include "imu.h"
#include "vars.h"
#include "config.h"
#include "nano_ball.h"
#include "position.h"

int zone[3][3];

void increaseIndex(int i, int j, int ammount){
    if(i < 3 && j < 3){
      zone[i][j] += ammount;
      zone[i][j] = constrain(zone[i][j], 0, ZONE_MAX_VALUE);
    }
}

void decreaseIndex(int i, int j, int ammount){
  increaseIndex(i, j, -ammount);
}

void increaseRow(int i, int ammount){
    if(i < 3){
        for(int a = 0; a < 3; a++){
            increaseIndex(a, i, ammount);
        }
    }
}

void decreaseRow(int i, int ammount){
  increaseRow(i, -ammount);
}

void increaseCol(int i, int ammount){
    if(i < 3){
        for(int a = 0; a < 3; a++){
            increaseIndex(i, a, ammount);
        }
    }
}

void decreaseCol(int i, int ammount){
  increaseCol(i, -ammount);
}

void increaseColWithLimit(int i, int ammount){
  if(zone[i][0] + ammount < ZONE_MAX_VALUE && zone[i][1] + ammount < ZONE_MAX_VALUE && zone[i][2] + ammount < ZONE_MAX_VALUE){
    increaseCol(i, ammount);
  }
}

void increaseRowWithLimit(int i, int ammount){
  if(zone[0][1] + ammount < ZONE_MAX_VALUE && zone[1][i] + ammount < ZONE_MAX_VALUE && zone[2][i] + ammount < ZONE_MAX_VALUE){
    increaseRow(i, ammount);
  }
}

void decreaseColWithLimit(int i, int ammount){
  if(zone[i][0] - ammount >= 0 && zone[i][1] - ammount >= 0 && zone[i][2] - ammount >= 0){
    decreaseCol(i, ammount);
  }
}

void decreaseRowWithLimit(int i, int ammount){
  if(zone[0][i] - ammount >= 0 && zone[1][i] - ammount >= 0 && zone[2][i] - ammount >= 0){
    decreaseRow(i, ammount);
  }
}

void increaseAll(int val){
    //decrease all
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            increaseIndex(i, j, val);
        }
    }
}

void decreaseAll(int val){
  increaseAll(-val);
}

void calculateLogicZone(){
    decreaseAll(ZONE_LOOP_DECREASE_VALUE);

    readPhyZone();
    //calculates guessed_x and guessed_y and zoneIndex
    //zoneIndex is just 2D to 1D of the guessed x and y
    //(y_position * width + x_position)

    int top = 0;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(zone[i][j] > top){
                guessed_x = i;
                guessed_y = j;
                top = zone[i][j];
            }
        }
    }
    zoneIndex = guessed_y * 3 + guessed_x;
}

void readPhyZone(){
  phyZoneUS();
  phyZoneCam();
  phyZoneLines();
  // phyZoneDirection();
}

//old WhereAmI. Renamed to be coerent. Now also adds to the logic zone
void phyZoneUS(){
  // decide la posizione in orizzontale e verticale
  // Aggiorna i flag :  good_field_x  good_field_y      non utilizzata da altre
  // routines
  //                    goal_zone                       non utilizzata da altre
  //                    routines
  // Aggiorna le variabili:
  //                    status_x (con valori CENTER EAST  WEST 255  = non lo so)
  //                    status_y (con valori CENTER NORTH SOUTH   255  = non lo so)

  int Lx_mis; // larghezza totale stimata dalle misure
  int Ly_mis; // lunghezza totale stimata dalle misure
  int Ly_min; // Limite inferiore con cui confrontare la misura y
  int Ly_max; // Limite inferiore con cui confrontare la misura y
  int Dy;     // Limite per decidere NORTH SOUTH in funzione della posizione
              // orizzontale

  old_status_x = status_x;
  old_status_y = status_y;
  good_field_x = false; // non é buona x
  good_field_y = false; // non é buona y
  goal_zone = false;    // non sono davanti alla porta avversaria

  if (role == HIGH)
    DxF = DxF_Atk;
  else
    DxF = DxF_Def;

  Lx_mis = us_dx + us_sx + robot; // larghezza totale stimata
  Ly_mis = us_fr + us_px + robot; // lunghezza totale stimata

  // controllo orizzontale
  if ((Lx_mis < Lx_max) && (Lx_mis > Lx_min) && (us_dx > 25) && (us_sx > 25)) {
    // se la misura orizzontale é accettabile
    good_field_x = true;
    status_x = CENTER;
    if (us_dx < DxF) // robot é vicino al bordo dEASTro
      status_x = EAST;
    if (us_sx < DxF) // robot é vicino al bordo sinistro
      status_x = WEST;

    if (status_x == CENTER) {
      // imposto limiti di controllo lunghezza verticale tra le porte
      Ly_min = LyP_min;
      Ly_max = LyP_max;
      Dy = DyP;
    } else {
      // imposto limiti di controllo lunghezza verticale in fascia
      Ly_min = LyF_min;
      Ly_max = LyF_max;
      Dy = DyF;
    }
  } else {
    // la misura non é pulita per la presenza di un ostacolo
    if ((us_dx >= (DxF + 10)) || (us_sx >= (DxF + 10))) {
      // se ho abbastanza spazio a dEASTra o a sinistra
      // devo stare per forza al cento
      status_x = CENTER;
      // imposto limiti di controllo lunghezza verticale tra le porte
      Ly_min = LyP_min;
      Ly_max = LyP_max;
      Dy = DyP;
    } else {
      status_x = 255;
      // non so la coordinata x
      // imposto i limiti di controllo verticale in base alla posizione
      // orizzontale precedente
      if (old_status_x == CENTER) {
        // controlla la posizione precedente per decidere limiti di controllo y
        // imposto limiti di controllo lunghezza verticale tra le porte
        Ly_min = LyP_min;
        Ly_max = LyP_max;
        Dy = DyP;
      } else {
        // imposto limiti di controllo lunghezza verticale in fascia anche per x
        // incognita
        Ly_min = LyF_min;
        Ly_max = LyF_max;
        Dy = DyF;
      }
    }
  }
  // controllo verticale
  if ((Ly_mis < Ly_max) && (Ly_mis > Ly_min)) {
    // se la misura verticale é accettabile
    good_field_y = true;
    status_y = CENTER;
    if (us_fr < Dy) {
      status_y = NORTH; // robot é vicino alla porta avversaria
      if (Dy == DyP)
        goal_zone = true; //  davanti alla porta in zona goal
    }
    if (us_px < Dy)
      status_y = SOUTH; // robot é vicino alla propria porta
  } else {
    // la misura non é pulita per la presenza di un ostacolo
    status_y = 255; // non so la coordinata y
    if (us_fr >= (Dy + 0))
      status_y = CENTER; // ma se ho abbastanza spazio dietro o avanti
    if (us_px >= (Dy + 0))
      status_y = CENTER; //  e'probabile che stia al CENTER
  }

  //now operates on the matrix
  if (status_x == 255 && status_y != 255) {
      increaseRow(status_y, ZONE_US_UNKNOWN_INCREASE_VALUE);
  } else if (status_x != 255 && status_y == 255) {
      increaseCol(status_x, ZONE_US_UNKNOWN_INCREASE_VALUE);
  } else {
      increaseIndex(status_x, status_y, ZONE_US_INDEX_INCREASE_VALUE);
  }
}

int p = 4;
int camA;
int camD;

void phyZoneCam(){

  //IMU-fixed attack angle
  camA = fixCamIMU(pAtk);
  //IMU-fixed defence angle
  camD = fixCamIMU(pDef);

  //Negative angle means that the robot is positioned to the right of the goalpost
  //Positive angle means that the robot is positioned to the left of the goalpost

  p = 4;

  if(abs(diff(camA, camD)) <= ZONE_CAM_CENTER_RANGE){
    //at center row, you can consider both camA and camD
      p = 1;
  }else if(camA > camD){
      p = 0;
  }else if(camD > camA){
      p = 2;
  }

  increaseColWithLimit(p, ZONE_CAM_INCREASE_VALUE);

  calcPhyZoneCam = false;
}



void phyZoneLines(){
  //ZONE_LINES_ERROR_VALUE is a random error code not used in line exit direction calculations
  if(lineSensByteBak != ZONE_LINES_ERROR_VALUE){ 
    switch(lineSensByteBak) {
      case 1:         //NORTH
        increaseRow(0, ZONE_LINES_INCREASE_VALUE);
        decreaseRow(1, ZONE_LINES_INCREASE_VALUE);
        decreaseRow(2, ZONE_LINES_INCREASE_VALUE);
      break;

      case 2:         //EAST
        decreaseCol(0, ZONE_LINES_INCREASE_VALUE);
        decreaseCol(1, ZONE_LINES_INCREASE_VALUE);
        increaseCol(2, ZONE_LINES_INCREASE_VALUE);
      break;

      case 4:         //SOUTH
        decreaseRow(0, ZONE_LINES_INCREASE_VALUE);
        decreaseRow(1, ZONE_LINES_INCREASE_VALUE);
        decreaseRow(2, ZONE_LINES_INCREASE_VALUE);
      break;

      case 8:         //WEST
        increaseCol(0, ZONE_LINES_INCREASE_VALUE);
        decreaseCol(1, ZONE_LINES_INCREASE_VALUE);
        decreaseCol(2, ZONE_LINES_INCREASE_VALUE);
      break;

      case 3:
        decreaseAll(ZONE_LINES_INCREASE_VALUE);
        increaseIndex(0, 2, 2*ZONE_LINES_INCREASE_VALUE);
      break;

      case 6:
        decreaseAll(ZONE_LINES_INCREASE_VALUE);
        increaseIndex(2, 2, 2*ZONE_LINES_INCREASE_VALUE);
      break;

      case 9:
        decreaseAll(ZONE_LINES_INCREASE_VALUE);
        increaseIndex(0, 0, 2*ZONE_LINES_INCREASE_VALUE);
      break;
        
      default:
      break;
    }
    //Last thing to do, sets the var to an error code, so next time it will be called will be because of the outOfBounds function being called
    lineSensByteBak = ZONE_LINES_ERROR_VALUE;
  }
}

void phyZoneDirection(){
  int val = 5;

  int x = sin(prevPidDir) * prevPidSpeed;
  int y = cos(prevPidSpeed) * prevPidSpeed;

  if(y > 0) decreaseRow(2, val);
  if(y < 0) decreaseRow(0, val);
  if(x > 0) increaseCol(2, val);
  if(x < 0) increaseCol(0, val);
}


void testPhyZone(){
    readBallNano();

    readUS();
    readIMU();
    goalPosition();

    readPhyZone();
    DEBUG_PRINT.print("Measured US location:\t");
    DEBUG_PRINT.print(status_x);
    DEBUG_PRINT.print(" | ");
    DEBUG_PRINT.println(status_y);
    DEBUG_PRINT.print("Measured Cam Column (4 is error):\t");
    DEBUG_PRINT.println(p);
}

void testLogicZone(){
    calculateLogicZone();
    DEBUG_PRINT.println("-----------------");

    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
        DEBUG_PRINT.print(zone[i][j]);
        DEBUG_PRINT.print(" | ");
        }
        DEBUG_PRINT.println();
    }
    DEBUG_PRINT.println("-----------------");
    DEBUG_PRINT.print("Guessed location:\t");
    DEBUG_PRINT.print(guessed_x);
    DEBUG_PRINT.print(" | ");
    DEBUG_PRINT.println(guessed_y);
    DEBUG_PRINT.print("Zone Index: ");
    DEBUG_PRINT.println(zoneIndex);
}

unsigned long ao;
void gigaTestZone(){

    //outpus the matrix
    if (millis() - ao >= 500) {
        DEBUG_PRINT.println("------");
        for (int i = 0; i < 4; i++) {
        DEBUG_PRINT.print("US: ");
        DEBUG_PRINT.print(us_values[i]);
        DEBUG_PRINT.print(" | ");
    }
    DEBUG_PRINT.println();
    testPhyZone();
    testLogicZone();
    testIMU();

    // if (comrade){
    //   DEBUG_PRINT.print("FriendZone: ");
    //   DEBUG_PRINT.println(friendZone);
    // }
    DEBUG_PRINT.println("------");
    ao = millis();
  }
}

void goCenter() {
  if (zoneIndex == 8)
    preparePID(330, GOCENTER_VEL);
  if (zoneIndex == 7)
    preparePID(0, GOCENTER_VEL);
  if (zoneIndex == 6)
    preparePID(45, GOCENTER_VEL);
  if (zoneIndex == 5)
    preparePID(270, GOCENTER_VEL);
  if (zoneIndex == 4)
    preparePID(0, 0);
  if (zoneIndex == 3)
    preparePID(90, GOCENTER_VEL);
  if (zoneIndex == 2)
    preparePID(255, GOCENTER_VEL);
  if (zoneIndex == 1)
    preparePID(180, GOCENTER_VEL);
  if (zoneIndex == 0)
    preparePID(135, GOCENTER_VEL);
}

void centerGoalPostCamera(bool checkBack) {
  digitalWrite(Y, HIGH);
  if (fixCamIMU(pDef) > CENTERGOALPOST_CAM_MAX) {
    preparePID(270, CENTERGOALPOST_VEL1);
  } else if (fixCamIMU(pDef) < CENTERGOALPOST_CAM_MIN) {
    preparePID(90, CENTERGOALPOST_VEL1);
  }else if(fixCamIMU(pDef) > CENTERGOALPOST_CAM_MIN && fixCamIMU(pDef) < CENTERGOALPOST_CAM_MAX){
    if(!ball_seen) preparePID(0, 0, 0);
    if(checkBack){
      if(us_px > CENTERGOALPOST_US_MAX){
        preparePID(180, CENTERGOALPOST_VEL2);
      } else{
        if(us_px < CENTERGOALPOST_US_CRITIC) preparePID(0, CENTERGOALPOST_VEL3);

        keeper_backToGoalPost = false;
        keeper_tookTimer = false;
      }
    }
  }
}

// int vel = 160;
// int usDist = 70;
// void centerGoalPost() {
//   x = 1;
//   y = 1;
//   int vel = 255;
//   if ((zoneIndex >= 0 && zoneIndex <= 2) || zoneIndex == 4) {
//     preparePID(180, vel);
//   } else if (zoneIndex == 3 || zoneIndex == 6) {
//     preparePID(90, vel);
//   } else if (zoneIndex == 5 || zoneIndex == 8) {
//     preparePID(270, vel);
//   } else {
//     stop_menamoli = false;
//     if (us_px >= 25)
//       preparePID(180, vel);
//     else
//       preparePID(0, 0);
//   }
// }

void update_sensors_all() {
  readBallNano();
  readIMU();
  readUS();
  return;
}

void AAANGOLO() {
  if((us_px <= 45) && ((us_dx <= 50) || (us_sx <= 50))) preparePID(0, 350, 0);
}