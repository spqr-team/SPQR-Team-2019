#include <Arduino.h>

#ifndef MAIN
#define extr extern
#else
#define extr
#endif

#define R 20
#define Y 17
#define G 13

// IR shield pin
#define BUZZER 30
#define SWITCH_SX 28
#define SWITCH_DX 29

// ZONE DEL CAMPO
// codici utilizzabili per una matice 3x3
#define EST 2
#define OVEST 0
#define CENTRO 1
#define NORD 0
#define SUD 2

#define NORD_OVEST 1
#define NORD_CENTRO 2
#define NORD_EST 3
#define CENTRO_OVEST 4
#define CENTRO_CENTRO 5 // codici zona nuovi
#define CENTRO_EST 6
#define SUD_OVEST 7
#define SUD_CENTRO 8
#define SUD_EST 9

// VARIABILI E COSTANTI DEL PID
#define KP 1.2          //2     // K proporzionale
#define KI 0            //0.1   // K integrativo
#define KD 0.7          //1.7   // K derivativo

// Linesensors e interrupt
#define S1I A14
#define S1O A15
#define S2I A16
#define S2O A17
#define S3I A20
#define S3O A0
#define S4I A1
#define S4O A2

#define LINE_THRESH 90
#define EXTIME 100
extr bool bounds;
extr bool slow;
extr elapsedMillis exitTimer;

extr int LN1I;
extr int LN2I;
extr int LN3I;
extr int LN4I;
extr int LN1O;
extr int LN2O;
extr int LN3O;
extr int LN4O;

#define BNO055_SAMPLERATE_DELAY_MS (60)

#define BT Serial3
#define DEBUG_PRINT Serial
#define CAMERA Serial2
#define NANO_BALL Serial4


// IMU
extr int imu_current_euler;

// Line Sensors
extr byte lineReading;

//Read ball
extr int ball_degrees, ball_distance;
extr bool ball_seen;

// PID
extr float errorePre;
extr float integral;
extr int st;
extr int prevPidDir;
extr int prevPidSpeed;
// test new angle
extr int globalDir;
extr int globalSpeed;
// Motors
extr float vx, vy, speed1, speed2, speed3, speed4, pidfactor, sins[360], cosin[360];
extr int x, y;

// US
extr int reading;
extr long us_t0;                     // US measure start
extr long us_t1;                     // time value during measure
extr bool us_flag;                   // is it measuring or not?
extr int us_values[4];               // US values array
extr int us_sx, us_dx, us_px, us_fr; // copies with other names in the array

// POSITION
extr int old_status_x;  // posizione precedente nel campo vale EST, OVEST o CENTRO o 255 >USI FUTURI<
extr int old_status_y;  // posizione precedente nel campo vale SUD, NORD o CENTRO o 255 >USI FUTURI<
extr bool good_field_x; // vedo tutta la larghezza del campo si/no
extr bool good_field_y; // vedo tutta la lunghezza del campo si/no
extr int status_x;      // posizione nel campo vale EST, OVEST o CENTRO o 255
extr int status_y;      // posizione nel campo vale SUD, NORD o CENTRO o 255
extr int guessed_x, guessed_y;
extr int zoneIndex;
extr bool calcPhyZoneCam;

extr byte lineSensByteBak;

extr int DxF;      // con  misura y OK e robot a EST o A OVEST con us_fx o us_px < DyF sto a NORD o a SUD  era - 10
extr bool goal_zone;

// BLUETOOTH
extr int a;
// puzzi tanto
extr unsigned long old_timer;

// Comunicazione compagno
extr int iAmHere;
extr int friendZone;            //    ;(
extr int fpos;
extr int role;
extr bool comrade;
extr int topolino;

// attack
extr int atk_direction;
extr int atk_speed;

// defense
extr bool defGoRight;
extr bool defGoLeft;
extr bool defGoBehind;
extr bool stop_menamoli;

extr elapsedMillis keeperAttackTimer;
extr bool keeper_tookTimer;
extr bool keeper_backToGoalPost;

extr int pAtk; // variabile dello switch che decide dove bisogna attaccare
extr int pDef; // variabile dello switch che decide dove bisogna difendere
extr bool XP_SX;
extr bool XP_DX;
extr int portx;
extr int goal_orientation;
extr int cameraReady;

extr float stincr;
extr float cstorc;


// test vars
extr char test; // test select
extr bool flagtest;