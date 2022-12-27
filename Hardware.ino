#include "LiquidCrystal.h"
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
LiquidCrystal lcd( 8,  9,  4,  5,  6,  7);
#include <TimerOne.h>
 RF24 radio(2, 10);  // CE, CSN
//address through which two modules communicate.
RF24Network network(radio);      // Include the radio in the network
const uint16_t this_node = 01;   // Address of our node in Octal format ( 04,031, etc)
const uint16_t master00 = 00;    // Address of the other node in Octal format

int pin = 3;
float rads = 57.29577951; // 1 radian = approx 57 deg.
float degree = 360;
float frequency = 50;
float nano = 1 * pow (10,-6); // Multiplication factor to convert nano seconds into seconds
float pf;
float angle;
float pf_max = 0;
float angle_max = 0;
int ctr;
const unsigned long Ts=2000;  //Sampling rate in microseconds(must be less than execution time of the program)
const unsigned char N=10;     //Number of samples to perform DFT on
unsigned char n;              //pointer for sample number
float factor;                 //DFT multiplication factor(calculated in initialization)
float FR;
float t;
float energy;
float energy1;
float P=0;
float S=0;
double sumWH = 0.00000;
float WH = 0;
double sumEGP= 0.00000;
float EGP = 0;
float theta1;
float theta2;
float theta;

float FI;                     //variable for storing imaginary part of DFT
float mag;
float FR1;                     //Variable for storing real part of DFT
float FI1;                     //variable for storing imaginary part of DFT
float mag1;     //variable for storing magnitude
float ctable[N];                  //cosine lookup table  
float stable[N];                 //sine lookup table
float s[N];
float k[N];//stored samples
float V=0;
float I=0;

void timerIsr();
int D1;
int D2;
/*long milisec = millis(); // calculate time in milliseconds
long t=milisec/1000; // convert milliseconds to seconds*/

void setup() {
  Serial.begin(9600);
 lcd.begin(16, 2);
 for (n=0;n<N;n++)            //creation of sine and cosine lookup tables
  {
  ctable[n]=cos(2.0*PI*(float)n/(float)N);
  stable[n]=sin(2.0*PI*(float)n/(float)N);
  }
  factor=2.0/(float)N;         //calculation of dft factor
  //analogReference(DEFAULT);   //maximum analogue input set from external source on AREF pin
  Timer1.initialize(Ts); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  pinMode(pin, INPUT);
 SPI.begin();
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);

 
}
void loop() {
 int x;
 x = analogRead (0);
 V=mag/1.414;
 if(V<2)
  {
    V=0;
  }
 I=mag1/1.414;
  if(I<0.1)
  {
    I=0;
  }
  S=V*I;

pf= abs(cos(theta*PI/180));
   P=V*I* pf;
   WH=P/3600;
sumWH = sumWH + WH;


  lcd.clear();
   
 Serial.print("volt = ");
  Serial.print(V );
  Serial.println("V");
 
radio.write(&P, sizeof(P));
radio.write(&sumWH, sizeof(sumWH));
  Serial.print("Write ret: ");

  delay(1000);

  /*lcd.clear();
   
Serial.print("Apparent Power  = ");
  Serial.print(S);
  Serial.println("VA");
  lcd.print("Apparent Power= ");
  lcd.setCursor(0,1);
lcd.print(S);
  lcd.println("VA");
  delay(1000);*/
 

    
 
 
 
     lcd.clear();
  lcd.setCursor(0,0);
 lcd.print("V=");
 lcd.print(V);
  lcd.print("V");
  

   lcd.setCursor(0,1);
lcd.print("I= ");
lcd.print(I);
lcd.print("A");
delay(3000);
 
   lcd.clear();
  lcd.setCursor(0,0);
 lcd.print("Angle=");
 lcd.print(theta);
  

   lcd.setCursor(0,1);
lcd.print("PF= ");
lcd.print(pf);
delay(3000);
 
 
  lcd.clear();
  lcd.setCursor(0,0);
 lcd.print("P=");
 lcd.print(P);
  lcd.print("W");

   lcd.setCursor(0,1);
lcd.print("E=");
 lcd.print(sumWH);
  lcd.print("Watt.Hour");

delay(3000);
RF24NetworkHeader header(master00);   // (Address where the data is going)
    network.write(header, &P, sizeof(P)); // Send the data
        network.write(header, &sumWH, sizeof(sumWH)); // Send the data

}
 


void timerIsr(){
  
D1=analogRead(A1);
 s[N-1]=((float)(1.01344*(D1))-518.88)*1.189; 
//s[N-1]=((float)(1.01344*(D1))-518.88);//capture value and rescale
 D2=analogRead(A3);
  
//k[N-1]=((float)(0.068986*(D2))-35.321)*1.12; 
k[N-1]=((float)(0.068986*(D2))-35.321)*1.174;
   FR=s[0]; //set initial value for real part
FI=0;  //set initial value for imaginary part
for (n=1;n<N;n++)  //calculate DFT
  {
  FR=FR+s[n]*ctable[n]; 
  FI=FI+s[n]*stable[n];
  s[n-1]=s[n];  //shift sample values
  }
  FR1=k[0]; //set initial value for real part
   FI1=0;  //set initial value for imaginary part
for (n=1;n<N;n++)  //calculate DFT
  {
  FR1=FR1+k[n]*ctable[n]; 
  FI1=FI1+k[n]*stable[n];
  k[n-1]=k[n];  //shift sample values
  }
mag=factor*sqrt(sq(FR)+sq(FI));//calculate magnitude
mag1=factor*sqrt(sq(FR1)+sq(FI1));//calculate magnitude
theta1 = atan (FR/FI)*(180/PI);
theta2 = atan (FR1/FI1)*(180/PI);
theta =abs(theta1-theta2);
}
