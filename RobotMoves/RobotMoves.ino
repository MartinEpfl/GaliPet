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
int lowlim =70; //Position when the arm is at the bottom
int upspeed = 1;
int downspeed = 10;
int setspeed = 10;
int waitingOnBottleTime = 1000; //Le temps attendu sur la bouteille

Servo servoBack;
int pinServoBack = 7; //Pin of servo for the back (PWM)
int positionOfBack;
int uplim_b = 70; //Position of back when close
int lowlim_b = 0; //Position of back when open
int speedBack = 20; //Speed back is opening/closing
int waitingBottleOut = 3000; //Waiting for bottle to go out


int E1 = 4;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)
int E2 = 5; //M2 Speed Control (PWM)
int M2 = 29; //M2 Direction control (Digital)
int speedForward = 150; //Speed moving forward between 0 and 255
int speedBackward = 150; //Speed moving backward between 0 and 255
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
  for (int positionA = positionOfArm; positionA >= uplim; positionA--) {
        servoArm.write(positionA);
        delay(setspeed);
  } 
  Serial.println("DONE");
  positionOfBack = servoBack.read(); 
  Serial.println("Reseting the back...");
  servoBack.write(uplim_b);
  for (int positionB = positionOfBack; positionB <= uplim_b; positionB++) {
        servoBack.write(positionB);
        delay(setspeed);
  } 
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
        back_off (speedBackward, speedBackward);   //move back in max speed
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

void arm(){
  stop();
   Serial.println("Arm Turning...");
   for (int position = uplim; position < lowlim; position++) {  
     servoArm.write(position);
     delay(downspeed);
   }          
   delay(waitingOnBottleTime);
   for (int position = lowlim; position > uplim; position--) {
     servoArm.write(position);
     delay(upspeed);
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
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);      
  digitalWrite(M2,HIGH);

}
void back_off (char a, char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
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
