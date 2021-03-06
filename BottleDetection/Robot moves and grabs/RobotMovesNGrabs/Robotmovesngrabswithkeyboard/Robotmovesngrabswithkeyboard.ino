#include <SharpIR.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
//#include <Pixy2UART.h>

//Commandes :
//w => Avancer
//s => Reculer
//a => Gauche
//d  => Droite
//b => Ouvrir le dos (fait arreter le robot)
//l => Faire baisser le bras (fait arreter le robot)

//////////////////////////////////////////////////////////////    DECLARATIONS   //////////////////////////////////////////////////////////////

byte incomingByte; //Byte being read from user
//All the speeds are in ms/angle (it is not a speed I know it's the inverse of a speed


Servo servoArm;
int pinservoArm =  8; //Pin of servo for the ARM (PWM)
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
int pinServoBack = 9; //Pin of servo for the back (PWM)
int positionOfBack;
int uplim_b = 110; //Position of back when close
int lowlim_b = 0; //Position of back when open
int speedBack = 25; //Speed back is opening/closing
int waitingBottleOut = 3000; //Waiting for bottle to go out

//Motors, M1 is left wheel, M2 is right wheel
int E1 = 6;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)
int E2 = 7; //M2 Speed Control (PWM)
int M2 = 29; //M2 Direction control (Digital)
//int speedForward = 30 ; //Speed moving forward cm/s
int speedBackward = 50; //Speed moving backward cm/s
int speedTurning = 30; //Speed while turning cm/s

const int r = 30;
const double sizeBetweenWheels = 38.3;
double speedForward = r;//(r-sizeBetweenWheels/2)*(PI/4);
const int optimalSpeedUpper = (r + sizeBetweenWheels / 2) * (PI / 4);
const int optimalSpeedLower = (r - sizeBetweenWheels / 2) * (PI / 4);



double diameterWheels = 12; //cm
double gearRatio = 74.83; //gear ratio of our pololu;
double countsPerRevolution = 48;
const double factorPulseToDistance = PI * diameterWheels / (countsPerRevolution*gearRatio);

//Motors Encodeurs


//Left wheel
const byte leftEncoder0pinA =  3;//Pin for left motor Encodeur  (must be a pin to use interrupt)
const byte leftEncoder0pinB = 4;//Second pin
double durationLeft = 0;
double speedWheelLeft = 0;
Encoder leftEncoder(leftEncoder0pinA, leftEncoder0pinB);

//Right wheel
const byte rightEncoder0pinA =  2;//first Pin for right motor Encodeur (must be a pin to use interrupt)
const byte rightEncoder0pinB = 5;//Second pin
double durationRight = 0;
double speedWheelRight = 0;
Encoder rightEncoder(rightEncoder0pinA, rightEncoder0pinB);

//Time between each loop
unsigned long previousTime = millis();
unsigned long currentTime = millis();
unsigned long diffTime = currentTime - previousTime;

//Robot position (odometry)
double x = 0; //Initialize position
double y = 0;
double angle = PI / 2; //Initial Angle (delta)
double phi = 0;
double distanceLeft = speedWheelLeft * diffTime; //Distance travelled by the left wheel
double distanceRight = speedWheelRight * diffTime;
double distanceCenter = (distanceLeft + distanceRight) / 2; //By the center of the robot (between the two wheels)
double distanceSinceBeginning = 0;
//double sizeBetweenWheels = 38.3; //Distance between the two wheels (TODO => CHANGE)


//PID for motor control
double d = 2;
//For the left motor
double targetSpeedLeft = 0; //
double pwmOutLeft = 0;
PID leftPID(&speedWheelLeft, &pwmOutLeft, &targetSpeedLeft, d, 15, 0.005, DIRECT);

//For the right motor
double targetSpeedRight = 0;
double pwmOutRight = 0;
PID rightPID(&speedWheelRight, &pwmOutRight, &targetSpeedRight, d, 15, 0.005, DIRECT);

double valueFromCompass;
/*

  //Pixy camera
  Pixy2UART pixy;
  float focalLengthHeight = 232.50; //Maybe we need to be more precise
  float focalLengthWidth = 267.15;
  int pixelsWidth;   //read by the camera
  int pixelsHeight; //read by the camera
  float distanceWidth;   //calculated distance based on the width of the object
  float distanceHeight;  //calculated distance based on the height of the object
  float averageDistance; //Average of both distances
  float widthOfObject = 13.7; //cm  Real size in cm of the object
  float heightOfObject = 4; //cm
  const int sizeOfArray = 20;
  double distancesAverage[sizeOfArray];
  double tempDistanceAverage[sizeOfArray];
  double value; //Actual value
  //float distance_ = 30; To use if we want to calibrate the focal length

*/

//SENSORS

const int window_size = 10;

#define model80 1080
#define model30 430
#define pin A13

struct sensor {
  double read_value = 0;
  double values_sensor[window_size];
  int index = 0;
  double average = 0;
  int pinSensor;
  SharpIR* mySensor;

  void sensorInit(int whichPin, int maxDistance) { //Max distance for the model, 30 if 30 cm max models, 80 otherwise
    pinSensor = whichPin;
    if (maxDistance == 30) {
      mySensor = new SharpIR(pinSensor, model30);
    }
    else if (maxDistance == 80) {
      mySensor = new SharpIR(pinSensor, model80);
    }
    else {
      mySensor = new SharpIR(pinSensor, model30);
    }

  }
  void loop_sensor() {
    read_value =  mySensor->distance();
    values_sensor[index] = read_value;
    index++;
    index = index % window_size;
  }

  double get_value() {
    average = 0;
    for (int i = 0; i < window_size; i++) {
      average += values_sensor[i];
    }
    return average / window_size;
  }
};

//Back sensors
const int numberOfSensorsBack = 2;

sensor sensorsBack[numberOfSensorsBack];
int pinsBack[] = {12, 13}; //Analog Pins for the back sensors
double thresholdBackSensors = 15; //The value in cm before the robot stops going backward if there is an obstacle at less than this distance.

//Front Sensors
const int numberOfSensorsFront = 4;

sensor sensorsFront[numberOfSensorsFront];
int pinsFront[] = {A1, A2, A3, A4}; //The sensors from 0 to 3 are left, middle, right, top according to the robots pov
int cutOffDistance = 100; //Value used to check if there is something in front of the sensor (in a binary way)
int veryCloseDistance = 35; //Value used to check if something is very close to the IR sensors

//////////////////////////////////////////////////////////////    SETUP   //////////////////////////////////////////////////////////////

void setup(void)
{
  Serial.begin(19200);      //Set Baud Rate
  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2, OUTPUT);

  //setting up servos
  servoArm.attach(pinservoArm);
  servoBack.attach(pinServoBack);
  positionOfArm = servoArm.read();
  Serial.println("Reseting the arm...");
  servoArm.writeMicroseconds(500);
  Serial.println("DONE");
  positionOfBack = servoBack.read();
  Serial.println("Reseting the back...");
  servoBack.write(uplim_b);
  Serial.println("DONE");


  // PIDs on
  leftPID.SetOutputLimits(0, 255);
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(5);
  rightPID.SetOutputLimits(0, 255);
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(5);

  //Sensors ON
  for (int i = 0; i < numberOfSensorsBack; i++) {
    sensorsBack[i].sensorInit(pinsBack[i], 30);
  }
  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].sensorInit(pinsFront[i], 80);
  }

  Serial3.begin(9600); //Compass has a Baud Rate of 9600

  // pixy.init();

  Serial.println("Controls :");
  Serial.println("w to advance.");
  Serial.println("s to back off.");
  Serial.println("a to go left.");
  Serial.println("d to go right.");
  Serial.println("l to make the arm go down.");
  Serial.println("d to open the back.");
  Serial.println("Run keyboard control");

}

//////////////////////////////////////////////////////////////    LOOP   //////////////////////////////////////////////////////////////

int acc = 0;
void loop(void)
{


  previousTime = currentTime;
  currentTime = millis();
  diffTime = currentTime - previousTime;

  durationLeft = abs(leftEncoder.read()); //Reads the left accumulated encodeur
  durationRight = abs(rightEncoder.read()); //Reads the value accumulated on the right encodeur
  speedWheelLeft = 1000 * factorPulseToDistance * durationLeft / diffTime; //  cm/s
  speedWheelRight = 1000 * factorPulseToDistance * durationRight / diffTime; //  cm/s
  odometry();
  leftEncoder.write(0); //Resets the accumulators to 0
  rightEncoder.write(0);

  /*
    Serial.print(pwmOutRight);
    Serial.print("  ");
    Serial.print(speedWheelRight);
    Serial.print("  ");
    Serial.print(pwmOutLeft);
    Serial.print("  ");
    Serial.println(speedWheelLeft);
  */
  Serial.print(sensorsFront[0].get_value());
  Serial.print("  ");
  Serial.print(sensorsFront[1].get_value());
  Serial.print("  ");
  Serial.print(sensorsFront[2].get_value());
  Serial.print("  ");
  Serial.println(sensorsFront[3].get_value());


  //  pixyRead();

  delay(5); //Needed because otherwise our loop function goes too fast
  // advance (speedForward, speedForward);   //move forward in max speed
  /*
    acc++;
    acc = acc % 5;
    if (acc == 0) {
      readValueCompass();
    }
  */
  if (Serial.available()) {
    char val = Serial.read();
    if (val != -1)
    {
      switch (val)
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
          turn_L (optimalSpeedLower, optimalSpeedUpper);
          break;
        case 'd'://Turn Right
          turn_R (optimalSpeedUpper, optimalSpeedLower);
          break;
        case 'l'://Arm goes down
          arm();
          break;
        case 'b'://Back Opens Up
          back();
          break;
        case 'p':
          speedForward += 10;

          Serial.println("Speed : ");
          Serial.println(speedForward);
          advance (speedForward, speedForward);
          break;
        case 'o':
          speedForward -= 10;

          Serial.println("Speed : ");
          Serial.println(speedForward);
          advance (speedForward, speedForward);
          break;

        case 'x':
          stop();
          break;
        case '+':
          leftPID.SetTunings(leftPID.GetKp() + 0.5, leftPID.GetKi(), leftPID.GetKd());
          Serial.print("leftPID P value is now : ");
          Serial.println(leftPID.GetKp());
          break;
        case '-':
          leftPID.SetTunings(leftPID.GetKp() - 0.5, leftPID.GetKi(), leftPID.GetKd());
          Serial.print("leftPID P value is now : ");
          Serial.println(leftPID.GetKp());
          break;
        case '6':
          leftPID.SetTunings(leftPID.GetKp(), leftPID.GetKi(), leftPID.GetKd() + 0.001);
          Serial.print("leftPID D value is now : ");
          Serial.println(leftPID.GetKd());
          break;
        case '9':
          leftPID.SetTunings(leftPID.GetKp(), leftPID.GetKi(), leftPID.GetKd() - 0.001);
          Serial.print("leftPID D value is now : ");
          Serial.println(leftPID.GetKd());
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

  leftPID.Compute();
  rightPID.Compute();
  analogWrite(E1, pwmOutLeft);
  analogWrite(E2, pwmOutRight);

  // Attempt to detect bottles
  bottleDetection();

}

//////////////////////////////////////////////////////////////    OTHER FUNCTIONS   //////////////////////////////////////////////////////////////

void readValueCompass() {
  char valeurByte[8];
  int stack = 0;
  boolean readByte = false;
  boolean value = false;
  double tempAngleCompass;
  Serial3.write(0x31); //Asking for the angle, for each command sent you get 8 byte as an answer
  //First byte, enter => New Line => hundreds of angle => tens of angle => bits of angle => Decimal point of angle => Decimal of angle => Calibrate sum
  while (!value) {
    if (Serial3.available()) {
      valeurByte[stack] = Serial3.read(); //Read the value & stacks it
      stack = (stack + 1) % 8; //Allows to read the full 8 bytes
      if (stack == 0) {
        tempAngleCompass = (valeurByte[2] - 48) * 100 + (valeurByte[3] - 48) * 10 + (valeurByte[4] - 48); //Computes the angle by reading bytes
        value = true;
      }
    }
  }
  valueFromCompass = tempAngleCompass;

  //        Serial.print("THIS IS THE ANGLE FROM THE COMPASS VALUE : ");
  Serial.println(valueFromCompass);
}


//--------------------------------------ODOMETRY
double totalDistance = 0;
void odometry() {
  if (digitalRead(M1) == HIGH) {
    distanceLeft = factorPulseToDistance * durationLeft; //Distance travelled by the left wheel
    totalDistance += distanceLeft;

  }
  else {
    distanceLeft = -factorPulseToDistance * durationLeft; //Distance travelled by the left wheel
  }
  if (digitalRead(M2) == LOW) {
    distanceRight = factorPulseToDistance * durationRight; //Distance travelled by the right wheel
  }
  else {
    distanceRight = -factorPulseToDistance * durationRight; //Distance travelled by the right wheel
  }

  distanceCenter = (distanceLeft + distanceRight) / 2;
  phi = (distanceRight - distanceLeft) / sizeBetweenWheels;
  x = x + distanceCenter * cos(angle);
  y = y + distanceCenter * sin(angle);
  angle = angle + phi; //New angle for our robot, to calibrate with the compass
  /*
    Serial.print("This is x position :");
    Serial.print(x);
    Serial.print(" and this y position :");
    Serial.print(y);
    Serial.print(" and this this the angle delta : ");
    Serial.println(angle);*/
}
/*
  //--------------------------------------PIXYREAD

  void pixyRead(){
  // grab blocks!
  pixy.ccc.getBlocks();

  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {

    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      pixelsWidth = pixy.ccc.blocks[i].m_width;
      pixelsHeight = pixy.ccc.blocks[i].m_height;
      distanceWidth = (focalLengthWidth * widthOfObject) /pixelsWidth;
      distanceHeight = (focalLengthHeight * heightOfObject) /pixelsHeight;
      averageDistance = (distanceWidth +distanceHeight)/2;
      for(int j = 1;j<sizeOfArray;j++){
        tempDistanceAverage[j] = distancesAverage[j-1];
      }
      tempDistanceAverage[0] = averageDistance;
      for(int j = 0;j<sizeOfArray;j++){
        distancesAverage[j] = tempDistanceAverage[j];
        value += distancesAverage[j];
      }
      value/=sizeOfArray;
      Serial.print("This is the value: ");
      Serial.println(value);
    }
  }
  }
*/

void refreshAllPID() {
  leftPID.Compute();
  rightPID.Compute();
  analogWrite(E1, pwmOutLeft);
  analogWrite(E2, pwmOutRight);
}


void bottleDetection() {

  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].loop_sensor();
  }


  if (sensorsFront[3].get_value() > 1.5*cutOffDistance) { //No obstacle detected --> T=0
    if (sensorsFront[1].get_value() > cutOffDistance) { //Nothing on middle sensor --> T=0 M=0
      if (sensorsFront[0].get_value() > cutOffDistance) { // Nothing on left sensor --> T=0 M=0 L=0
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=0 L=0 R=1
          if (sensorsFront[2].get_value() < veryCloseDistance) { // Right sensor detects --> T=0 M=0 L=0 R=1 but it's closer than focal point
            Serial.println("Turns right fast");
            advance(speedForward * 0.6, speedForward * 0.2); //Turns right fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }
          else {
            Serial.println("Turns left slowly");
            advance(speedForward * 0.4, speedForward * 0.6); //Turns left slowly
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }

        }
        else {
          //Do nothing ==> // Nothing detected --> T=0 M=0 L=0 R=0
        }
      }
      else { // Left sensor detects --> T=0 M=0 L=1
        if (sensorsFront[2].get_value() > cutOffDistance) { // Nothing on right sensor --> T=0 M=0 L=1 R=0
          if (sensorsFront[0].get_value() < veryCloseDistance) { // Only left sensor detects --> T=0 M=0 L=1 R=0 but it's closer than focal point
            Serial.println("Turns left fast");
            advance(speedForward * 0.2, speedForward * 0.6); //Turns left fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }
          else {
            Serial.println("Turns right slowly");
            advance(speedForward * 0.6, speedForward * 0.4); //Turns right slowly
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }

        }
      }
    }
    else { //Middle sensor detects --> T=0 M=1
      if (sensorsFront[0].get_value() < cutOffDistance) { // Left sensor detects --> T=0 M=1 L=1
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=1 L=1 R=1
          Serial.println("Goes forward for a bit");
          advance(speedForward * 0.3, speedForward * 0.3); //Goes forward for a bit
          for (int i = 0; i < 15; i++) {
            delay(10);
            refreshAllPID();
          }
          delay(1000);
          Serial.println("Grabs bottle");
          stop();
          for (int i = 0; i < 15; i++) {
            delay(10);
            refreshAllPID();
          }
          arm();                                          //Grabs bottle
          advance(speedForward * 0.5, speedForward * 0.5);
        }
        else { // Nothing on right detector --> T=0 M=1 L=1 R=0
          if (sensorsFront[0].get_value() < veryCloseDistance) { // Left sensor detects --> T=0 M=1 L=1 R=0 but it's closer than focal point
            Serial.println("Turns left fast");
            advance(speedForward * 0.2, speedForward * 0.6); //Turns left fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }
          else {
            Serial.println("Turns right fast");
            advance(speedForward * 0.6, speedForward * 0.2); //Turns right fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }

        }
      }
      else { // Nothing on left sensor --> T=0 M=1 L=0
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=1 L=0 R=1
          if (sensorsFront[2].get_value() < veryCloseDistance) { // Right sensor detects --> T=0 M=1 L=0 R=1 but it's closer than focal point
            Serial.println("Turns right fast");
            advance(speedForward * 0.6, speedForward * 0.2); //Turns right fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }
          else {
            Serial.println("Turns left fast");
            advance(speedForward * 0.2, speedForward * 0.6); //Turns left fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              refreshAllPID();
            }
          }
        }
        else { // Nothing on Right sensor --> T=0 M=1 L=0 R=0
          Serial.println("Goes forward");
          advance(speedForward * 0.5, speedForward * 0.5); //Goes forward
          for (int i = 0; i < 15; i++) {
            delay(10);
            refreshAllPID();
          }
        }
      }
    }
  }
  else { // Top sensor detects --> T=1
    Serial.println("Obstacle detected");
    stop();
    for (int i = 0; i < 15; i++) {
      delay(10);
      refreshAllPID();
    }
  }
}


//--------------------------------------ARM

void arm() {
  stop();
  Serial.println("Arm Turning...");
  for (int position = uplim; position < lowlim; position++) {
    servoArm.writeMicroseconds(position);
    delay(downspeed);
  }
  delay(waitingOnBottleTime);
  for (int position = lowlim; position > intermediatePosition; position--) {
    servoArm.writeMicroseconds(position);
    delay(upspeedFirstPart);
  }
  for (int position = intermediatePosition; position > uplim; position -= 2) {
    servoArm.writeMicroseconds(position);
    delay(upspeedSecondPart);
  }

  Serial.println("-DONE TURNING-");
}

//--------------------------------------BACK

void back() {
  stop();
  Serial.println("Back opening...");
  for (int position = uplim_b; position > lowlim_b; position--) {
    servoBack.write(position);
    Serial.println(servoBack.read());

    delay(speedBack);
  }
  delay(waitingBottleOut);
  for (int position = lowlim_b; position < uplim_b; position++) {
    servoBack.write(position);
    Serial.println(servoBack.read());

    delay(speedBack);
  }
  Serial.println("-DONE OPENING/CLOSING-");
}

//--------------------------------------STOP

void stop(void)  //Stop
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  targetSpeedLeft = 0;
  targetSpeedRight = 0;
  digitalWrite(E1, 0);
  digitalWrite(E2, 0);
}

//--------------------------------------ADVANCE

void advance(char a, char b)  //Move forward
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  targetSpeedLeft = a;
  targetSpeedRight = b;

  //analogWrite (E1,a);      //PWM Speed Control
  //analogWrite (E2,b);


}

//--------------------------------------BACK OFF

void back_off (char a, char b) //Move backward
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);

  targetSpeedLeft = a;
  targetSpeedRight = b;

  //analogWrite (E1,a);
  //analogWrite (E2,b);

}

//--------------------------------------TURN LEFT

void turn_L (char a, char b) //Turn Left
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  targetSpeedLeft = a;
  targetSpeedRight = b;

  //analogWrite (E1,a);
  //analogWrite (E2,b);

}


//--------------------------------------TURN RIGHT

void turn_R (char a, char b) //Turn Right
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  targetSpeedLeft = a;
  targetSpeedRight = b;

  //analogWrite (E1,a);
  //analogWrite (E2,b);

}
