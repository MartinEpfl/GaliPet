#include <Encoder.h>
#include <Servo.h>
//TODO => CHECK POSITION DE REPOS POUR LE BACK, 180 OU 0? 
//En fonction faut changer le setup et la fonction back
//TODO => Voir si 3 secondes (variable waitingBottleOut)suffisent au robot pour sortir les bouteilles et si la
//vitesse est bonne (du dos)
//TODO => Verifier les vitesse du bras, il devrait etre plus rapide pour monter que pour descendre.

//Commandes : 
//w => Avancer
//s => Reculer
//a => Gauche
//d  => Droite
//b => Ouvrir le dos (fait arreter le robot)
//l => Faire baisser le bras (fait arreter le robot)

byte incomingByte; //Byte being read from user
//All the speeds are in ms/angle (it is not a speed I know it's the inverse of a speed


Servo servoArm;
int pinservoArm =  8; //Pin of servo for the ARM (PWM)
int positionOfArm;
int uplim = 3; //Position when the arm is at the top
int lowlim =300; //Position when the arm is at the bottom
int upspeedFirstPart = 10;
int upspeedSecondPart = 1;
int downspeed = 10;
int setspeed = 10;
int waitingOnBottleTime = 1000; //Le temps attendu sur la bouteille
int intermediatePosition = 30;

Servo servoBack;
int pinServoBack = 9; //Pin of servo for the back (PWM)
int positionOfBack;
int uplim_b = 70; //Position of back when close
int lowlim_b = 0; //Position of back when open
int speedBack = 20; //Speed back is opening/closing
int waitingBottleOut = 3000; //Waiting for bottle to go out

//Motors, M1 is left wheel, M2 is right wheel
int E1 = 6;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)
int E2 = 7; //M2 Speed Control (PWM)
int M2 = 29; //M2 Direction control (Digital)
int speedForward = 100; //Speed moving forward between 0 and 255
int speedBackward = 150; //Speed moving backward between 0 and 255
double diameterWheels = 24; //cm
double gearRatio = 74.83; //gear ratio of our pololu;
double countsPerRevolution = 48;
const double factorPulseToSpeed = 1000*PI*diameterWheels/(countsPerRevolution*gearRatio);

//Motors Encodeurs


//Left wheel
const byte leftEncoder0pinA =  3;//Pin for left motor Encodeur  (must be a pin to use interrupt)
const byte leftEncoder0pinB = 4;//Second pin
double durationLeft = 0;
double speedWheelLeft = 0;
Encoder leftEncoder(leftEncoder0pinA,leftEncoder0pinB);

//Right wheel
const byte rightEncoder0pinA =  2;//first Pin for right motor Encodeur (must be a pin to use interrupt)
const byte rightEncoder0pinB = 5;//Second pin
double durationRight = 0;
double speedWheelRight = 0;

Encoder rightEncoder(rightEncoder0pinA,rightEncoder0pinB);

//Time between each loop
unsigned long previousTime = millis();
unsigned long currentTime = millis();
unsigned long diffTime = currentTime - previousTime;

//Robot position (odometry)
double x = 0; //Initialize position
double y = 0;
double angle = PI/2; //Initial Angle (delta)
double phi = 0;
double distanceLeft = speedWheelLeft * diffTime; //Distance travelled by the left wheel
double distanceRight = speedWheelRight * diffTime;
double distanceCenter = (distanceLeft + distanceRight)/2; //By the center of the robot (between the two wheels)
double sizeBetweenWheels = 40; //Distance between the two wheels (TODO => CHANGE)


void setup(void)
{
  Serial.begin(57600);      //Set Baud Rate
  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2, OUTPUT);
  servoArm.attach(pinservoArm);
  servoBack.attach(pinServoBack);
  positionOfArm = servoArm.read(); 
  Serial.println("Reseting the arm...");
  armInit();
  Serial.println("DONE");
  positionOfBack = servoBack.read(); 
  Serial.println("Reseting the back...");
  servoBack.write(uplim_b);
  Serial.println("DONE");
  Serial.println("Controls :");
  Serial.println("w to advance.");
  Serial.println("s to back off.");
  Serial.println("a to go left.");
  Serial.println("d to go right.");
  Serial.println("l to make the arm go down.");
  Serial.println("d to open the back.");
  Serial.println("Run keyboard control");

}

void loop(void)
{
  previousTime = currentTime;
  currentTime = millis();
  diffTime = currentTime - previousTime;
  durationLeft = leftEncoder.read(); //Reads the left accumulated encodeur
  durationRight = rightEncoder.read(); //Reads the value accumulated on the right encodeur
  speedWheelLeft = factorPulseToSpeed*durationLeft/diffTime; //  cm/ms
  speedWheelRight = factorPulseToSpeed*durationRight/diffTime;//  cm/ms
  odometry();
  leftEncoder.write(0); //Resets the accumulators to 0
  rightEncoder.write(0);
  delay(100); //Needed because otherwise our loop function goes too fast
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        Serial.println("Move forward");
        advance (speedForward, speedForward);   //move forward in max speed
        break;
      case 's'://Move Backward
        Serial.println("Move backward");
        back_off (speedBackward, speedForward);   //move back in max speed
        break;
      case 'a'://Turn Left
        turn_L (100,100);
        break;
      case 'd'://Turn Right
        turn_R (100,100);
        break;
      case 'l'://Arm goes down
        arm();
        break;
      case 'b'://Back Opens Up
        back();
        break;
      case 'p':
        speedForward+=10;

        Serial.println("Speed : ");
        Serial.println(speedForward);
        advance (speedForward, speedForward);
        break;
      case 'o':
        speedForward-=10;
        
        Serial.println("Speed : ");
        Serial.println(speedForward);
        advance (speedForward, speedForward);
        break;
      
      case 'x':
        stop();
        break;
      }
    }
    else stop();
    Serial.println("Controls : W to advance.");
    Serial.println("s to back off.");
    Serial.println("a to go left.");
    Serial.println("d to go right.");
    Serial.println("l to make the arm go down.");
    Serial.println("d to open the back.");
    Serial.println("Run keyboard control");
  }

}
void odometry(){
  distanceLeft = speedWheelLeft * diffTime; //Distance travelled by the left wheel
  distanceRight = speedWheelRight * diffTime;
  distanceCenter = (distanceLeft + distanceRight)/2;    
  phi = (distanceRight - distanceLeft)/sizeBetweenWheels;
  x = x + distanceCenter*cos(angle);
  y = y + distanceCenter*sin(angle);
  angle = angle + phi; //New angle for our robot, to calibrate with the compass
  
}

void arm(){
  stop();
   Serial.println("Arm Turning...");
   for (int position = uplim; position < lowlim; position++) {  
     servoArm.write(position);
     delay(downspeed);
   }          
   delay(waitingOnBottleTime);
   for (int position = lowlim; position > intermediatePosition; position--) {
     servoArm.write(position);
     delay(upspeedFirstPart);
   }
   for (int position = intermediatePosition; position > uplim; position--) {
     servoArm.write(position);
     delay(upspeedSecondPart);
   }
   Serial.println("-DONE TURNING-");
}

void back(){
   stop();
   Serial.println("Back opening...");
   for (int position = uplim_b; position > lowlim_b; position--) {  
     servoBack.write(position);
     delay(speedBack);
   }          
   delay(waitingBottleOut);
   for (int position = lowlim_b; position < uplim_b; position++) {
     servoBack.write(position);
     delay(speedBack);
   }   
   Serial.println("-DONE OPENING/CLOSING-");   
}
void armInit(){
  for (int positionA = positionOfArm; positionA >= uplim; positionA--) {
        servoArm.write(positionA);
        delay(setspeed);
  } 
}


void stop(void)                    //Stop
{
  digitalWrite(E1,0);
  digitalWrite(M1,LOW);
  digitalWrite(E2,0);
  digitalWrite(M2,LOW);
}
void advance(char a, char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,LOW);
  analogWrite (E2,b);      
  digitalWrite(M2,HIGH);

}
void back_off (char a, char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}

void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}

void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
