#define SPI_SS 4
#define SPI_MOSI 5
#define SPI_MISO 6
#define SPI_SCK 7

#define SENS0 (PINC & 128) //PC7
#define SENS1 (PINC & 64) //PC6
#define SENS2 (PINC & 32)//PC5
#define SENS3 (PINC & 16) //PC4
#define SENS4 (PINC & 8) //PC3
#define SENS5 (PINC & 4) //PC2
#define SENS6 (PINC & 2) //PC1
#define SENS7 (PINC & 1) //PC0

#define SENS8 (PIND & 128) //PD7
#define SENS9 (PIND & 64) //PD6
#define SENS10 (PIND & 32) //PD5
#define SENS11 (PIND & 16) //PD4


#define SENS12 (PINA & 1) //PA0
#define SENS13 (PINA & 2) //PA1
#define SENS14 (PINA & 4) //PA2
#define SENS15 (PINA & 8)//PA3
#define SENS16 (PINA & 16) //PA4
#define SENS17 (PINA & 32) //PA5
#define SENS18 (PINA & 64) //PA6
#define SENS19 (PINA & 128) //PA7

#define SOGLIA_SENSOREROTTO 230
#define SOGLIA_PALLAINVISTA 35
#define SOGLIA_PALLA0 200 // attaccato ma non sempre funziona   
#define SOGLIA_PALLA1 170 // circa 5-15cm
#define SOGLIA_PALLA2 150 // circa 15-80cm
#define SOGLIA_PALLA3 125 //  circa 80-140cm
#define SOGLIA_PALLA4 100  // circa oltre 140cm
#define SOGLIA_PALLA5 75   // lontanissima   
#define SOGLIA_PALLA6 7     //palla non vista
#define LETTURE 250

#include <avr/interrupt.h>

int contatore[20];    //array contenete conteggi sensori palla durante acquisizione dati
// sensori         0  1  2    3   4   5   6   7    8    9    10   11   12   13   14   15   16   17   18   19
int angoli[20] = { 
  0, 15, 30, 45, 60, 75, 90, 113, 135, 158, 180, 202, 225, 247, 270, 285, 300, 315, 330, 345};
byte sensmax = 0;
byte distanza = 0;
volatile byte infoball = 0;
volatile byte intcalled = 0;
volatile byte oldinfoball = 0;

void setup() {
  pinMode(SPI_MOSI, INPUT);
  pinMode(SPI_MISO, OUTPUT);
  pinMode(SPI_SCK, INPUT);
  pinMode(SPI_SS, INPUT);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  MCUCR = MCUCR | 128;
  DDRC=0;
  DDRA=0;
  DDRD&=15;  
  //Serial.begin(9600);
  Serial1.begin(9600);//seriale per debug xbee
  SPCR |= 64; // abilito SPI
// SPCR |= 192; //SLAVE E INTERRUPTS
//  sei();
}

byte osc = 0;

void loop() {
//  Serial1.println("ciao");
//  digitalWrite(1,osc);
//  osc ^=1;
  leggiSens();
//    Serial1.write(255);
//    Serial1.write(infoball);
//    for(int i=0;i<20;i++)
//    {
//      Serial1.write(byte(contatore[i]/10));
//    }
//    Serial1.write(254);
}

void leggiSens() {
  //intcalled = 0;
  for(int i = 0;i<20;i++)contatore[i]=0;
  for(int i = 0; i<LETTURE;i++){
    contatore[0] +=  (!SENS0);
    contatore[1] +=  (!SENS1);
    contatore[2] +=  (!SENS2);
    contatore[3] +=  (!SENS3);
    contatore[4] +=  (!SENS4);
    contatore[5] +=  (!SENS5);
    contatore[6] +=  (!SENS6);
    contatore[7] +=  (!SENS7);
    contatore[8] +=  (!SENS8);
    contatore[9] +=  (!SENS9);
    contatore[10] += (!SENS10);
    contatore[11] += (!SENS11);
    contatore[12] += (!SENS12);
    contatore[13] += (!SENS13);
    contatore[14] += (!SENS14);
    contatore[15] += (!SENS15);
    contatore[16] += (!SENS16);
    contatore[17] += (!SENS17);
    contatore[18] += (!SENS18);
    contatore[19] += (!SENS19);
  }
//  if(contatore[0] == LETTURE)sensmax=1; //se il sensore zero ÃƒÂ¨ rotto incomincio a confrontare con il sensore numero 1
  for (byte i = 0; i < 20; i++)   //confronto il valore di ogni sensore con quello piÃƒÂ¹ alto, in modo da stabilire qual'ÃƒÂ¨ realmente il piÃƒÂ¹ alto fra i 20
  {
    if ((contatore[i] > contatore[sensmax])&& (contatore[i] != LETTURE)) // ignoro il dato se un sensore e' sempre attivo
      sensmax = i;
    }
      if (contatore[sensmax] < SOGLIA_PALLA6) { // sotto il valore di 60 = palla non in vista
      sensmax = 0; // se la palla non e' in vista uscita = 0;
      distanza = 6;
      digitalWrite(1,0);
      }
    else {
      digitalWrite(1,1); //toggle led palla in vista
      if (contatore[sensmax] < SOGLIA_PALLA0) distanza=0;
      if (contatore[sensmax] < SOGLIA_PALLA1)   distanza=1;
      if (contatore[sensmax] < SOGLIA_PALLA2)  distanza=2;
      if (contatore[sensmax] < SOGLIA_PALLA3) distanza=3;
      if (contatore[sensmax] < SOGLIA_PALLA4)   distanza=4;
      if (contatore[sensmax] < SOGLIA_PALLA5)  distanza=5;
      if (contatore[sensmax] < SOGLIA_PALLA6)  distanza=6;
    }
    distanza=distanza<<5;
    infoball=distanza | sensmax;
    oldinfoball = infoball;

    SPDR= infoball;
  }


//************************
ISR(SPI_STC_vect )
{
  intcalled = 1;
    SPCR = 64 & 127;  // disabilita interrupt
    SPDR= infoball;
   // SPDR=PINC;
    SPCR = 64 |128;  // abilita interrupt
}




