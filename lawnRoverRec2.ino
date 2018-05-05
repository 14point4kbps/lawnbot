
#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"



// Motor pins

#define LmotorA             2  // Left  motor H bridge, input A
#define LmotorB             3  // Left  motor H bridge, input B
#define RmotorA             5  // Right motor H bridge, input A
#define RmotorB             6  // Right motor H bridge, input B

int LeftPWMA; //PWM value for left motor speed / brake
int LeftPWMB; //PWM value for left motor speed / brake
int RightPWMA; //PWM value for right motor speed / brake
int RightPWMB; //PWM value for right motor speed / brake

int outPin = 49; //motor controller power relay 

// nRF24L01 radio
RF24 radio(9, 53);
const uint64_t pipes[2] =
{
	0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL
};


// Timer
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))


// Structs for communication
typedef struct
{
	int X;
	int Y;
	int Z;
	int A;
	int B;
	int C;
	int D;
}
struct1_t;
struct1_t remote;

typedef struct
{
	float motors;
	int batRover;
}
struct2_t;
struct2_t rover;

// analog pins
int analogPins[3] = {A0, A1, A2}; //A0 Bat, A1= Rght motor current, A2 = Lft motor current
int analogReadings[3];

// Other stuff...
byte current;
byte battery;
unsigned long startTime;
unsigned long endTime;
int time;

void setup()
{
	Serial.begin(115200);
	// Setup radio
	radio.begin();
	radio.setPALevel( RF24_PA_MAX ) ;
	radio.setDataRate(RF24_1MBPS);
	//radio.setDataRate(RF24_250KBPS);
	//radio.enableDynamicPayloads() ;
	//radio.setAutoAck(true);
	//radio.setPayloadSize(14);
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1, pipes[0]);
	radio.startListening();



//Output pins
     pinMode(outPin, OUTPUT); //motor controller relay pin 49
//  pinMode(redLed, OUTPUT);  // error led (red)
//  pinMode(orangeLeds, OUTPUT); // orange leds
// pinMode(headlight, OUTPUT); // white leds
// pinMode(blueLed, OUTPUT); // blue led
//  pinMode(buzzer, OUTPUT); // buzzer
//
}

void loop(void)
{
	startTime = millis();

	// Radio
	runEvery(75)     // running this code to often leads to dropped packets
	{
		//   Serial.println("listening");
		radio.stopListening();
		bool ok = radio.write( &rover, sizeof(rover) );
		radio.startListening();

		unsigned long started_waiting_at = millis();
		bool timeout = false;
		while ( ! radio.available() && ! timeout )
			//if (millis() - started_waiting_at > 1+(radio.getMaxTimeout()/1000) )
			if (millis() - started_waiting_at > 200 )
				timeout = true;

		if ( timeout )          // no wireless connection
		{
			Serial.println("Failed, response timed out.");
			stopped();

		}
		else
		{
			radio.read( &remote, sizeof(remote) );
		}
	}
	// end of radio stuff


// Analog reading: checking battery voltage and motor current
	for (int i = 0; i < 3; i += 1)
	{
		analogRead(analogPins[i]);                      // don't use first reading
		analogReadings[i] = analogRead(analogPins[i]);
	}

	// Add all motor currents (to sent to remote control)
	rover.motors = (analogReadings[1]);// + analogReadings[2]) / 2;
	rover.batRover = analogReadings[2];//((514- analogReadings[1])*36.76/1023);
	//rover.batRover = ((analogReadings[2] - 514) * 36.76 / 1023);
	// Check if a motor is stalled
	if (analogReadings[1] > 980 || analogReadings[2] > 980 || analogReadings[1] < 50 || analogReadings[2] < 50)//2 amps
	{
		current = LOW;
	  //  digitalWrite(outPin, LOW); //de-activate motor controller
	}
	else
	{
		Serial.print("analogReadings[0]");
		Serial.println(analogReadings[0]);
		Serial.print("analogReadings[1]");
		Serial.println(analogReadings[1]);
		Serial.print("analogReadings[2]");
		Serial.println(analogReadings[2]);
		current = HIGH;
		  digitalWrite(outPin, HIGH); //activate motor controller
	}

	/* // Check 2S Lipo battery voltage
	 rover.batRover = analogReadings[0] / 102.3;  // 1023 = 10V
	 if(rover.batRover > 7)
	 {
	   digitalWrite(redLed, LOW);
	   battery = HIGH;
	 }
	 else
	 {
	   battery = LOW;
	   digitalWrite(redLed, HIGH);
	 }
	*/
	// End of analog readings



	//current = HIGH; //temp
	battery = HIGH; //temp
	if(battery == HIGH && current == HIGH) //battery == ok, motor current == ok
	{
		if (remote.Z == LOW)   // driving mode
		{
			Serial.println("driving mode");

			if (remote.X >= 510 && remote.X <= 516 && remote.Y >= 485 && remote.Y <= 490 )    // joystick is centered
			{
				Serial.println("centered");
				stopped();
			}

			if (remote.Y > 480 && remote.Y < 505 && remote.X < 490)                  // hard left = left motors backward, right motors forward
			{
			
			
					LeftPWMA = 0; //
					LeftPWMB = 60; //
					RightPWMA = 60; //
					RightPWMB = 0; //
					//turnLeft();
					drive();
			}

			if (remote.Y > 480 && remote.Y < 505 && remote.X > 520)                  // hard right = left motors forward, right motors backward
			{
				LeftPWMA = 60; //
				LeftPWMB = 0; //
				RightPWMA = 0; //
				RightPWMB = 60; //
				//turnRight();
				drive();
			}

			if (remote.Y > 505 && remote.X > 480 && remote.X < 520)               // joystick forward = all motors forward
			{
					LeftPWMA = 0; // analogWrite(LmotorA, 0);
					LeftPWMB = 60; //  analogWrite(LmotorB, 80);
					RightPWMA = 0; // analogWrite(RmotorA, 0);
					RightPWMB = 60; //  analogWrite(RmotorB, 80);
					drive();
			}

			if (remote.Y < 485 && remote.X > 500 && remote.X < 520)               // joystick backward = all motors backward
			{
				LeftPWMA = 60;
				LeftPWMB = 0; //
				RightPWMA = 60; //
				RightPWMB = 0; //
				drive();
			}

			if (remote.Y > 495 && (remote.X < 505 || remote.X > 520))              // forward turning
			{
				if (remote.X < 510)
				{  					//left forward turning
					Serial.println("left");
					LeftPWMA = 0; //
					LeftPWMB = 60; //
					RightPWMA = 0; //
					RightPWMB = 30; //
					drive();
				}
				if (remote.X > 515)
				{					//right forward turning
					Serial.println("right");
					LeftPWMA = 0; //
					LeftPWMB = 30; //
					RightPWMA = 0; //
					RightPWMB = 60; //
					//   forward();
					drive();
				}
			}

			if (remote.Y < 480 && (remote.X < 516 || remote.X > 505))              // backward turning
			{
				if (remote.X < 505) // turn left reverse
				{
					LeftPWMA = 60;
					LeftPWMB = 0; 
					RightPWMA = 30;
					RightPWMB = 0;
					backward();
				}
				if (remote.X > 520) // turn right reverse
				{
					LeftPWMA = 60;
					LeftPWMB = 0;
					RightPWMA = 30;
					RightPWMB = 0;
					backward();
				}
			}


		} // end of driving mode

	}


	else  //starter motor z button pressed
	{
		Serial.print("remote.Z = ");
		Serial.println(remote.Z);
		notmove();
		//   blinkOrangeLeds();

	}




	endTime = millis();
	time = endTime - startTime;


	/* // serial printing
	runEvery(250)
	{
		Serial.print("remote.X = ");
		Serial.println(remote.X);
		Serial.print("remote.Y = ");
		Serial.println(remote.Y);
		Serial.print("remote.Z = ");
		Serial.println(remote.Z);
		// Serial.println(rover.batRover);
		Serial.print("time = ");
		Serial.println(time);
	   for (int i=0; i<2; i +=1)
		  {
		    Serial.print("PulseM = ");
		    Serial.println(PulseM[i]);
		    Serial.print("PWMM = ");
		    Serial.println(PWMM[i]);


		  }
		// end of serial printing
	}*/

}

/*void M1encoder()
{
  PulseM[0]=micros()-TimeM[0];              // time between last state change and this state change
  TimeM[0]=micros();                        // update TimeM[0] with time of most recent state change
}

void M2encoder()
{
  PulseM[1]=micros()-TimeM[1];
  TimeM[1]=micros();
}*/



void stopped()
{
//  for(int i=0; i<2; i++)
	{
//   PWMM[i] = 0;
//   analogWrite(PWMpinM[i], PWMM[i]);
		Serial.println("STOPPED");
		analogWrite(LmotorA, 0);
		analogWrite(LmotorB, 0);
		analogWrite(RmotorA, 0);
		analogWrite(RmotorB, 0);
	}
}
void notmove()
{
//  for(int i=0; i<2; i++)
	{
		Serial.println("not going");
	}
}

void drive()
{
	Serial.println("forward");
//  for(int i=0; i<2; i++)
	{
		analogWrite(LmotorA, LeftPWMA);
		analogWrite(LmotorB, LeftPWMB);
		analogWrite(RmotorA, RightPWMA);
		analogWrite(RmotorB, RightPWMB);
	}
//  digitalWrite(orangeLeds, LOW);
}

/*void forward()
{
  Serial.println("forward");
  {
    analogWrite(LmotorA, LeftPWMA);
    analogWrite(LmotorB, LeftPWMB);
    analogWrite(RmotorA, RightPWMA);
    analogWrite(RmotorB, RightPWMB);
  }
//  digitalWrite(orangeLeds, LOW);
}
*/

void backward()
{
	Serial.println("backward");

	{
		analogWrite(LmotorA, LeftPWMA);
		analogWrite(LmotorB, LeftPWMB);
		analogWrite(RmotorA, RightPWMA);
		analogWrite(RmotorB, RightPWMB);
	}
}

void turnLeft()
{
	Serial.println("left");
	{
		analogWrite(LmotorA, LeftPWMA);
		analogWrite(LmotorB, LeftPWMB);
		analogWrite(RmotorA, RightPWMA);
		analogWrite(RmotorB, RightPWMB);
	}
}

void turnRight()
{
	Serial.println("right");
	{
		analogWrite(LmotorA, LeftPWMA);
		analogWrite(LmotorB, LeftPWMB);
		analogWrite(RmotorA, RightPWMA);
		analogWrite(RmotorB, RightPWMB);
	}
}

/*void blinkOrangeLeds()
{
  runEvery(200)
  {
    if (orangeLedState == LOW)
      orangeLedState = HIGH;
    else
      orangeLedState = LOW;
  }
  digitalWrite(orangeLeds, orangeLedState);
}
*/
/*
void pwmMotors()
{
  for (int i=0; i<2; i++)
  {
    lastPulseM[i] = micros() - TimeM[i];
    if (lastPulseM[i] > targetPulse[i])
    {
      PWMM[i]+=2;
      if(PWMM[i] > 80)                   // Max pwm = 80
      {
        PWMM[i] = 80;
      }
    }
    if (lastPulseM[i] < targetPulse[i])
    {
      PWMM[i]-=2;
      if (PWMM[i] < 0)
      {
        PWMM[i] = 0;
      }
    }
  }
}
*/




