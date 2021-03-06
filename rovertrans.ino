

// http://www.bajdi.com
// Remote control
// Arduino Uno + joystick shield + I2C 4x20 LCD
// nRF24L01 wireless control 

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Bounce.h>
//#include <Bounce2.h>

#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

LiquidCrystal_I2C lcd(0x27,20,4); // set the LCD address to 0x3F

typedef struct{
  int X;
  int Y;
  int Z;
//  int A;
  int B;
  int C;
  int D;
}
struct1_t;

typedef struct{
  float motors;
  float batRover;
}
struct2_t;

struct1_t remote; 
struct2_t rover;

RF24 radio(9,10);
const uint64_t pipes[2] = { 
  0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

const int zAxis = 8;
//const int clawOpen = 3;
//const int clawClose = 6;
//const int topButton = 4;
//const int bottomButton = 5;
int zState;         
//float batRemote;
Bounce bouncerz = Bounce( zAxis, 10 );

void setup()
{
  Serial.begin(57600);
  pinMode(zAxis, INPUT);
  digitalWrite(zAxis, HIGH);
//  pinMode(clawOpen , INPUT);
//  digitalWrite(clawOpen, HIGH);
//  pinMode(clawClose , INPUT);
//  digitalWrite(clawClose, HIGH);
//  pinMode(topButton , INPUT);
//  digitalWrite(topButton, HIGH);
//  pinMode(bottomButton , INPUT);
//  digitalWrite(bottomButton, HIGH);
  radio.begin();
  radio.setPALevel( RF24_PA_MAX ) ; 
  //radio.setDataRate(RF24_250KBPS);
  //radio.setAutoAck( true ) ;
  radio.setDataRate(RF24_1MBPS);
  //radio.setPayloadSize(14);
  //radio.enableDynamicPayloads() ;
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();

 lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Motors=");
  lcd.setCursor(0,2);
  lcd.print("Bat remote = ");
  lcd.setCursor(0,3);
  lcd.print("Bat rover = ");

}

void loop(void)
{
 // analogRead(A3);
 // batRemote = analogRead(A3) / 102.3;
  analogRead(A6);
  remote.X = analogRead(A6);
  analogRead(A7);
  remote.Y = analogRead(A7);
/*  remote.A = digitalRead(clawOpen);    
  remote.B = digitalRead(clawClose);
  remote.C = digitalRead(topButton);  
  remote.D = digitalRead(bottomButton);
*/  
 
  if ( bouncerz.update() ) {
    if ( bouncerz.read() == LOW) {
      if ( remote.Z == HIGH ) {
        remote.Z = LOW;
      } 
      else {
        remote.Z = HIGH;
      }
    }
  }

  // radio stuff
  if ( radio.available() )
  {
    bool done = false;
    while (!done)
    {
      done = radio.read( &rover, sizeof(rover) );
    }
    radio.stopListening();
    radio.write( &remote, sizeof(remote) );
    radio.startListening();
  }
  // end of radio stuff

  //lcd stuff
  runEvery(200)
  {
  lcd.setCursor(8,0);
  lcd.print(rover.motors);
//  lcd.setCursor(13,2);
//  lcd.print(batRemote);
//  lcd.setCursor(12,3);
//  lcd.print(rover.batRover);
  }
  // end of lcd stuff
}

