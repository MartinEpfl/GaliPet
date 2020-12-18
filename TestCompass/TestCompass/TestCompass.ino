/*
  Gy-26 - Compass.
  Criado por Igor Araujo - www.igoraraujo.eng.br - 2012
*/

#include <SharpIR.h>
#include <Servo.h>


#define SWIRPin A4
#define model 1080

char valorbyte[8];
int graus = 0;
int contador = 0;
byte valor = 0;
double diff = 0;
double angle = 0;
double distanceNE;
SharpIR sensorNW = SharpIR(SWIRPin, model);

Servo servoArm;
int pinservoArm =  9; //Pin of servo for the ARM (PWM)
int positionOfArm;
int uplim = 500; //Position when the arm is at the top
int lowlim = 2700; //Position when the arm is at the bottom
int upspeedFirstPart = 3;
int upspeedSecondPart = 3;
int downspeed = 1;
int setspeed = 10;
int waitingOnBottleTime = 1000; //Time waited on the bottle
int intermediatePosition = 1000;


Servo servoBack;
int pinServoBack = 8; //Pin of servo for the back (PWM)
int positionOfBack;
int uplim_b = 110; //Position of back when close
int lowlim_b = 0; //Position of back when open
int speedBack = 25; //Speed back is opening/closing
int waitingBottleOut = 3000; //Waiting for bottle to go out


int E1 = 6;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)
int E2 = 7; //M2 Speed Control (PWM)
int M2 = 29; //M2 Direction control (Digital)
int speed_ = 125; //Speed moving backward cm/s

void setup() {
  //  servoArm.attach(pinservoArm);
  //servoArm.write(20);
  //servoBack.attach(pinServoBack);

  Serial.begin(9600);
  Serial2.begin(9600);
  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2, OUTPUT);

  readValueCompass();

  diff = PI / 4 - angle;
}

void loop() {
  distanceNE = sensorNW.distance();
  readValueCompass();
  //Serial.println(angle);
  if (Serial.available()) {
    char val = Serial.read();
    if (val != -1)
    {
      switch (val)
      {
        case 'w'://Move Forward
          // Serial.println("Move forward");
          advance (speed_, speed_);   //move forward in max speed
          break;
        case 'x'://Move Backward
          //Serial.println("STOP");
          stop();   //move back in max speed
          break;
      }
    }
  }
}

void readValueCompass() {
  char valeurByte[8];
  int stack = 0;
  boolean readByte = false;
  boolean value = false;
  double tempAngleCompass;
  Serial2.write(0x31); //Asking for the angle, for each command sent you get 8 byte as an answer
  //First byte, enter => New Line => hundreds of angle => tens of angle => bits of angle => Decimal point of angle => Decimal of angle => Calibrate sum
  while (!value) {
    //  Serial.println("stuck");
    if (Serial2.available()) {
      valeurByte[stack] = Serial2.read(); //Read the value & stacks it
      stack = (stack + 1) % 8; //Allows to read the full 8 bytes
      if (stack == 0) {
        tempAngleCompass = (valeurByte[2] - 48) * 100 + (valeurByte[3] - 48) * 10 + (valeurByte[4] - 48); //Computes the angle by reading bytes
        value = true;
      }
    }
  }

  //        Serial.print("THIS IS THE ANGLE FROM THE COMPASS VALUE : ");
  Serial.println(tempAngleCompass);
}

void stop(void)  //Stop
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(E1, 0);
  digitalWrite(E2, 0);
}


void advance(char a, char b)  //Move forward
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  analogWrite (E1, a);     //PWM Speed Control
  analogWrite (E2, b);


}
