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

// Note
#define LA3 220.00
#define C4 261.63
#define F3 174.61
#define E6 1318.51
#define F6 1396.91
#define GB6 1479.98

// ZONE DEL CAMPO // codici utilizzabili per una matice 3x3
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
#define KP 2   // K proporzionale
#define KI 0.1 // K integrativo
#define KD 0   // K derivativo
// #define KP 1.4
// #define KI 0.1
// #define KD 0.5
// SPI
#define SS_PIN 2
// Linesensors e interrupt
#define S1I A14
#define S1O A15
#define S2I A16
#define S2O A17
#define S3I A20
#define S3O A0
#define S4I A1
#define S4O A2
#define INT_LUNG 6

//extr int LINE_THRESH;
#define LINE_THRESH 150
extr int outDir;
extr int outVel;
extr bool bounds;
extr int EXTIME;

extr int LN1I;
extr int LN2I;
extr int LN3I;
extr int LN4I;
extr int LN1O;
extr int LN2O;
extr int LN3O;
extr int LN4O;

extr bool U0;
extr bool U1;
extr bool U2; // ;)
extr bool U3;

extr int Ux0, Uy0, Ux1, Uy1, Ux2, Uy2, Ux3, Uy3, Ux, Uy, U;

//turns to 0 the movement
extr int y;
extr int x;

#define BNO055_SAMPLERATE_DELAY_MS (60)

// COSTANTI PER ATTACCANTE (GOALIE & MENAMOLI)-----------------------------
#define GOALIE_MAX 130
#define GOALIE_MIN 200
#define GOALIE_SLOW1 130
#define GOALIE_SLOW2 150
#define GOALIE_DANGER 100
#define VEL_RET 180
#define GOALIE_P 255 // velocità portiere
#define AA0  0        // angoli di attacco in funzione del sensore palla
#define AA1  45
#define AA2  75
#define AA3  100 
#define AA4  135
#define AA5  165
#define AA6  175
#define AA7  210            //DIETRO
#define AA8  210            //DIETRO
#define AA9  150            //DIETRO
#define AA10 180
#define AA11 200
#define AA12 225
#define AA13 260
#define AA14 285
#define AA15 315

#define BT Serial3

#define DEBUG_PRINT Serial

#define CAMERA Serial2

#define NANO_BALL Serial4

// You can modify this if you need
// LIMITI DEL CAMPO
#define Lx_min 115  // valore minimo accettabile di larghezza
#define Lx_max 195  // valore massimo accettabile di larghezza (larghezza campo)
#define LyF_min 190 // valore minimo accettabile di lunghezza sulle fasce
#define LyF_max 270
// valore massimo accettabile di lunghezza sulle fasce (lunghezza campo)
#define LyP_min 139 // valore minimo accettabile di lunghezza tra le porte
#define LyP_max 250
// valore massimo accettabile di lunghezza tra le porte// con misura x OK
// con us_dx o us_sx < DxF sto nelle fasce 30 + 30 - 1/2
// robot

#define DxF_Atk 48 // per attaccante, fascia centrale allargata
#define DxF_Def 48 // per portiere, fascia centrale ristretta
// questa roba viene fatta dentro WhereAmI
extr int DxF;

// con  misura y OK e robot a EST o A OVEST con us_fx o us_px < DyF sto a
// NORD o a SUD  era - 10
#define DyF 91
// con misura y OK e robot al CENTRO (tra le porte) con us_fx o us_px < DyP
// sto a NORD o a SUD era - 22
// #define DyP 69
#define DyP 55
#define robot 21 // diametro del robot

// IMU
extr int imu_temp_euler, imu_current_euler;
// non serviiii
// Line Sensors
extr byte lineReading;
extr volatile bool flag_interrupt;
extr volatile byte
    nint; // numero di interrupt consecutivi prima della fine della gestione
extr volatile byte linea[INT_LUNG];
extr int VL_INT;    // velocitá di uscita dalle linee
extr int EXT_LINEA; // direzione di uscita dalla linea
extr byte n_us;     // ultrasuono da controllare dopo la fuga dalla linea
extr int attesa;    // tempo di attesa palla dopo un interrupt (utilizzata dallo
                    // switch destro)
extr bool danger;   // avviso che terminata da poco la gestione interrupt
extr unsigned long
    tdanger; // misura il tempo dal termine della gestione interrupt
// Motors
extr float vx, vy, speed1, speed2, speed3, speed4, pidfactor, sins[360], cosin[360];
// MySPI
extr byte mess, ball_sensor, ball_distance, old_s_ball, ball_degrees;
extr long time_s_ball, tspi;
extr bool ball_seen;
// PID
extr float errorePre;
// non servi a nullaaaaaa
// angolo di errore precedente
extr float integral;     // somisa degli angoli di errore
extr bool reaching_ball; // serve per aumentare il PID del 20% GOALIE
extr int st;             // storcimento sulle fasce
// da utilizzare per sviluppi futuri
extr signed int old_Dir; // angolo di direzione precedente a quella attuale
extr signed int new_Dir; // angolo di direzione corrente del moto
extr float old_vMot;     // velocitá di moto precedente
extr float new_vMot;     // velocitá di moto corrente
// US
extr int reading;
extr long us_t0;                     // US measure start
extr long us_t1;                     // time value during measure
extr bool us_flag;                   // is it measuring or not?
extr int us_values[4];               // US values array
extr int us_sx, us_dx, us_px, us_fr; // copies with other names in the array
// POSITION
extr int old_status_x;  // posizione precedente nel campo vale EST, OVEST o
                        // CENTRO o 255 >USI FUTURI<
extr int old_status_y;  // posizione precedente nel campo vale SUD, NORD o
                        // CENTRO o 255 >USI FUTURI<
extr bool goal_zone;    // sto al centro rispetto alle porte         assegnata// da
                        // WhereAmI ma non usata
extr bool good_field_x; // vedo tutta la larghezza del campo si/no
extr bool good_field_y; // vedo tutta la lunghezza del campo si/no
extr int status_x;      // posizione nel campo vale EST, OVEST o CENTRO o 255
extr int status_y;      // posizione nel campo vale SUD, NORD o CENTRO o 255
extr int guessed_x, guessed_y;
extr int zoneIndex;

// extr int currentlocation; // risultato misure zone campo da 1 a 9 o 255 se
//                           // undefined
// extr int guessedlocation; // risultato misure zone campo da 1 a 9 (da
// CENTRO_CENTRO a SUD_OVEST)
// extr int old_currentlocation; // zona precedente del robot in campo da 1 a 9
// o
//                               // 255 se undefined >USI FUTURI<
// extr int old_guessedlocation; // zona precedente del robot in campo da 1 a 9
// (da
//                               // CENTRO_CENTRO a SUD_OVEST) >USI FUTURI<
// extr byte zone[3][3];     // il primo indice = NORD SUD CENTRO  il secondo
//                           // indice  EST OVEST CENTRO
// signed int zone_prob[3][3];

// BLUETOOTH
extr int a;
// puzzi tanto
extr unsigned long old_timer;
// Interrupt
extr byte lineball_sensor;
extr byte lineball_distance;
extr int lineBallDir;
// extr float angle;
// extr int ldir, lspeed;

// Comunicazione compagno
extr int iAmHere;
extr int friendZone;
extr int role;
extr bool comrade;
extr int topolino;

// test new angle
extr int globalDir;
extr int globalSpeed;
// extr int st;

// attack
extr int atk_direction;
extr int atk_speed;
extr int atk_offset;
extr int Nint;
// defense
extr bool defGoRight;
extr bool defGoLeft;
extr bool defGoBehind;
extr bool stop_menamoli;

// variabili camera
#define centrop 160   // valore letto dalla camera come centro
#define keeperMin 60  // dx limit
#define keeperMax 240 // sx limit
// ema e' scemo da morire (letta)
// centro a 150
#define goalieCamMin 150
#define goalieCamMax 180

extr int pAtk; // variabile dello switch che decide dove bisogna attaccare
extr int pDef; // variabile dello switch che decide dove bisogna difendere
extr bool XP_SX;
extr bool XP_DX;
extr int portx;
extr int goal_orentation;
extr int cameraReady;

extr float stincr;

extr int fpos;

// test vars
extr char test; // test select
extr bool flagtest;
