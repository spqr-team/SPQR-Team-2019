//All defines go here

//Multiply the speed by this factor to make the robot go a bit slower or faster, without having to change all the variables manually
#define GLOBAL_SPD_MULT 0.9

//LINES
#define LINES_EXIT_SPD 350

//STRIKER
#define GOALIE_ATKSPD_LAT  255
#define GOALIE_ATKSPD_BAK  350
#define GOALIE_ATKSPD_FRT  345
#define GOALIE_ATKSPD_STRK 355
#define GOALIE_ATKDIR_PLUSANG1 20
#define GOALIE_ATKDIR_PLUSANG2 35
#define GOALIE_ATKDIR_PLUSANG3 40
#define GOALIE_ATKDIR_PLUSANGBAK 40
#define GOALIE_ATKDIR_PLUSANG1_COR 60
#define GOALIE_ATKDIR_PLUSANG2_COR 70
#define GOALIE_ATKDIR_PLUSANG3_COR 70

//KEEPER
#define KEEPER_ATTACK_DISTANCE 200
#define KEEPER_ALONE_ATTACK_TIME 5000 //in millis
#define KEEPER_COMRADE_ATTACK_TIME 3000//in millis
#define KEEPER_BASE_VEL 320
#define KEEPER_VEL_MULT 1.4
#define KEEPER_BALL_BACK_ANGLE 30

//POSITION
#define CENTERGOALPOST_VEL1 280
#define CENTERGOALPOST_VEL2 180
#define CENTERGOALPOST_CAM_MIN -30
#define CENTERGOALPOST_CAM_MAX 30
#define CENTERGOALPOST_US_MAX 45
#define CENTERGOALPOST_US_CRITIC 25
#define GOCENTER_VEL 280

#define ZONE_MAX_VALUE 150
#define ZONE_LOOP_DECREASE_VALUE 4
#define ZONE_US_UNKNOWN_INCREASE_VALUE 4
#define ZONE_US_INDEX_INCREASE_VALUE 9
#define ZONE_CAM_INCREASE_VALUE 3
#define ZONE_CAM_CENTER_RANGE 25
#define ZONE_LINES_INCREASE_VALUE 100
#define ZONE_LINES_ERROR_VALUE 30

// You can modify this if you need
// LIMITI DEL CAMPO
#define Lx_min 115  // valore minimo accettabile di larghezza
#define Lx_max 195  // valore massimo accettabile di larghezza (larghezza campo)
#define LyF_min 190 // valore minimo accettabile di lunghezza sulle fasce
#define LyF_max 270 // valore massimo accettabile di lunghezza sulle fasce (lunghezza campo)
#define LyP_min 139 // valore minimo accettabile di lunghezza tra le porte
#define LyP_max 250 // valore massimo accettabile di lunghezza tra le porte// con misura x OK con us_dx o us_sx < DxF sto nelle fasce 30 + 30 - 1/2 robot

#define DyF 91     // con misura y OK e robot al CENTER (tra le porte) con us_fx o us_px < DyP sto a NORTH o a SOUTH era - 22
#define DyP 69
#define DxF_Atk 48 // per attaccante, fascia centrale allargata
#define DxF_Def 48 // per portiere, fascia centrale ristretta quEASTa roba viene fatta dentro WhereAmI
#define robot 21   // diametro del robot


// ZONE DEL CAMPO
// codici utilizzabili per una matice 3x3
#define EAST 2
#define WEST 0
#define CENTER 1
#define NORTH 0
#define SOUTH 2

#define NORTH_WEST 1
#define NORTH_CENTER 2
#define NORTH_EAST 3
#define CENTER_WEST 4
#define CENTER_CENTER 5 // codici zona nuovi
#define CENTER_EAST 6
#define SOUTH_WEST 7
#define SOUTH_CENTER 8
#define SOUTH_EAST 9
