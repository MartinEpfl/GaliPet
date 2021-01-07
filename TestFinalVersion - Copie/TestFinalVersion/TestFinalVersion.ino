
#include <PID_v1.h>
#include <Encoder.h>
#include <Servo.h>
#include <SharpIR.h>


////////////////////////////// NAVIGUATION ///////////////////////////////////////////

typedef struct {
  double x;
  double y;
  boolean canGoThere = true;
  int howFar = 0;
} position_;






const int sizeBadAreaGrassX = 400;
const int sizeBadAreaGrassY = 200;
const int sizeBadAreaRockXY = 300;
const int sizeBadAreaUpperX = 300;
const int sizeBadAreaUpperY = 200;
const int greyArea = 150;
const int sizeOfFullArena = 800; //IF FULL ARENA

double epsilon = 20; //How close you dont want to get close to the area you don't want to go in
const double r = 40; //Radius of circle
position_ positionOfRobot; //Our robot


position_ leftRight[2]; //Possibilities for dodging the obsacle


position_ possibilities[3]; //Posibilities of where to go
const double angles[3] = {PI / 4, 0, -PI / 4,};


bool destinationAvailable = false; //If there is somewhere to go
boolean travellingToADestination = true; //If it is going somewhere
int indexPosibility = 1;
int count = 0;
const int maxIteration = 200;
bool robotIsHome = false;
bool goingBack = false; //If the robot is moving backward
bool wasGoingBack = false; //If the last movement was to go backward
bool goingHome = false; //If the robot is going home
bool isCurrentlydodgingObstacle = false; //If the robot is currently dodging an obstacle
bool goingToGetBottle = false; //If the robot is going to get a bootle
double ratioBeforeGoingHome = 0.75 ;
int totalFar = 0;
double valueX[4 * maxIteration];
double valueY[4 * maxIteration];


//Odometry
double speedWheelRight = 0; //cm/s Speed of left wheel
double speedWheelLeft = 0; //Speed of right wheel
double sizeBetweenWheels = 38.3; //cm
double sizeHeight = 41; //cm
double timeBetweenRead = 1;
double distanceLeft ;
double distanceRight;
double distanceCenter = (distanceLeft + distanceRight) / 2;
double phi = 0;
double currentAngle = PI / 4.0; //Starting angle


//All the speed for going forward/going back/left & right
const int optimalSpeedLower = (r - sizeBetweenWheels / 2) * (PI / 4) / 1.5;
const int optimalSpeedUpper = (r + sizeBetweenWheels / 2) * (PI / 4) / 1.5;
const int optimalSpeedForward = r / 1.5;
const int optimalSpeedBackward = optimalSpeedForward;
const int optimalSpeedTurn = sizeBetweenWheels * (PI / 4);


/////////Number of iteration for each movement (Each iteration is about X ms, so if time_ is 40 then the robot will do the move for 40*X ms/)
int time_ = 0;
const int time_forward = 40;
const int time_turn = 40;
const int time_dodge = 40;

/////////////////////////////////////////////////////
byte incomingByte; //Byte being read from user
//All the speeds are in ms/angle (it is not a speed I know it's the inverse of a speed

////////////////Compass//////////
double angleCompass = 0;
double initialDifference = 0;
double diffMax = 0;
double valueFromCompass;
double diff;
//////////////////////SENSORS//////////////////////////////////////

const int window_size = 5;

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
const int numberOfSensorsBack = 2;
const int numberOfSensorObstacle = 3;


sensor sensorsBack[numberOfSensorsBack];
sensor sensorsObstacle[numberOfSensorObstacle];

int pinsBack[] = {12, 13}; //Analog Pins for the back sensors
int pinsObstacle[] = {A4, A5, A6};


double thresholdBackSensors = 30; //The value in cm before the robot stops going backward if there is an obstacle at less than this distance.

const int numberOfSensorsFront = 3;
sensor sensorsFront[numberOfSensorsFront];
int pinsFront[] = {A1, A2, A3}; //The sensors from 0 to 3 are left, middle, right, top according to the robots pov
int cutOffDistance = 100; //Value used to check if there is something in front of the sensor (in a binary way)
int veryCloseDistance = 35; //Value used to check if something is very close to the IR sensors

///////Servo of the Arm////////////

Servo servoArm;
int pinservoArm =  8; //Pin of servo for the ARM (PWM)
int positionOfArm;
int uplim = 200; //Position when the arm is at the top
int lowlim = 3000; //Position when the arm is at the bottom
int upspeedFirstPart = 3;
int upspeedSecondPart = 1;
int downspeed = 1;
int setspeed = 10;
int waitingOnBottleTime = 0; //Time waited on the bottle
int intermediatePosition = 1000;
int numberOfBottles = 0;
const int numberMaxOfBottle = 5;
///////Servo of the back/////////
Servo servoBack;
int pinServoBack = 9; //Pin of servo for the back (PWM)
int positionOfBack;
int uplim_b = 45; //Position of back when close
int lowlim_b = 0; //Position of back when open
int speedBack = 50; //Speed back is opening/closing
int waitingBottleOut = 3000; //Waiting for bottle to go out

//Motors, M1 is left wheel, M2 is right wheel
int E1 = 6;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)
int E2 = 7; //M2 Speed Control (PWM)
int M2 = 29; //M2 Direction control (Digital)
int speedForward = 40 ; //Speed moving forward cm/s
int speedBackward = 50; //Speed moving backward cm/s
int speedTurning = 30; //Speed while turning cm/s
double diameterWheels = 12; //cm
double gearRatio = 74.83; //gear ratio of our pololu;
double countsPerRevolution = 48;
const double factorPulseToDistance = PI * diameterWheels / (countsPerRevolution*gearRatio);

//Motors Encodeurs


//Left wheel
const byte leftEncoder0pinA =  3;//Pin for left motor Encodeur  (must be a pin to use interrupt)
const byte leftEncoder0pinB = 4;//Second pin
double durationLeft = 0;
Encoder leftEncoder(leftEncoder0pinA, leftEncoder0pinB);

//Right wheel
const byte rightEncoder0pinA =  2;//first Pin for right motor Encodeur (must be a pin to use interrupt)
const byte rightEncoder0pinB = 5;//Second pin
double durationRight = 0;
Encoder rightEncoder(rightEncoder0pinA, rightEncoder0pinB);

//Time between each loop
unsigned long previousTime = millis();
unsigned long currentTime = millis();
unsigned long diffTime = currentTime - previousTime;
unsigned long timeBeforeDelay = millis();
unsigned long timeAfterDelay = millis();
float timeSinceBegin;
float timeBeforeGoingHome =  60; //In Seconds

//PID for motor control

//For the left motor
double targetSpeedLeft = 50; //
double pwmOutLeft = 0;
PID leftPID(&speedWheelLeft, &pwmOutLeft, &targetSpeedLeft, 5.1, 2, 0.005, DIRECT);

//For the right motor
double targetSpeedRight = 0;
double pwmOutRight = 0;
PID rightPID(&speedWheelRight, &pwmOutRight, &targetSpeedRight, 5.1, 2, 0.005, DIRECT);

unsigned actual = millis();
unsigned old = millis();
unsigned long tempTime = millis();

void setup() {
  timeSinceBegin = millis();
  // put your setup code here, to run once:
  Serial.begin(19200);      //Set Baud Rate
  // Serial2.begin(9600); //Compass has a Baud Rate of 9600
  randomSeed(analogRead(1));

  //readValueCompass();
  initialDifference = valueFromCompass - currentAngle ;
  diffMax = 0;
  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2, OUTPUT);
  positionOfRobot.x = 50;
  positionOfRobot.y = 50;
  //setting up servos
  servoBack.attach(pinServoBack);
  positionOfArm = servoArm.read();
  Serial.println("Reseting the arm...");
  servoArm.writeMicroseconds(500);
  Serial.println("DONE");
  positionOfBack = servoBack.read();
  Serial.println("Reseting the back...");
  for (int position = lowlim_b; position < uplim_b; position++) {
    servoBack.write(position);
    delay(1);

  }
  delay(100);
  // servoBack.detach();

  // PIDs on

  leftPID.SetOutputLimits(0, 255);
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(10);
  rightPID.SetOutputLimits(0, 255);
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(10);
  Serial.println(pwmOutLeft);
  Serial.println(pwmOutRight);


  //Sensors ON
  for (int i = 0; i < numberOfSensorsBack; i++) {
    sensorsBack[i].sensorInit(pinsBack[i], 30);
  }
  for (int i = 0; i < numberOfSensorObstacle; i++) {
    sensorsObstacle[i].sensorInit(pinsObstacle[i], 80);
  }
  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].sensorInit(pinsFront[i], 80);
  }
}

double compassArray[5];

void loop() {
  robotIsHome = true;

  Serial.println(pwmOutLeft);
  Serial.println(pwmOutRight);
  if (isnan(pwmOutLeft)) {
    analogWrite(E1, 0);
    pwmOutLeft = 0;
    Serial.println("NAN");
  }
  if (isnan(pwmOutRight)) {
    analogWrite(E2, 0);
    pwmOutLeft = 0;
    Serial.println("NAN");
  }
  refreshAllPID();
  old = actual;
  actual = millis();

  tempTime = actual - old;
  Serial.println(tempTime);
  timeSinceBegin = millis();
  if (Serial.available()) {
    char val = Serial.read();
    if (val != -1)
    {
      switch (val)
      {
        case 'x'://Move Forward
          stop();
          count = 100;
          robotIsHome = true;
          break;
      }
    }
    else stop();
  }
  if (count < maxIteration && !robotIsHome) {
    //  Serial.print("This is COUNT :");
    // Serial.println(count);
    if (time_ == 0) {
      epsilon += 0.2;

    }
    //UPDATING THE BACK SENSORS :
    for (int i = 0; i < numberOfSensorsBack; i++) {
      sensorsBack[i].loop_sensor();
    }
    for (int i = 0; i < numberOfSensorObstacle; i++) {
      sensorsObstacle[i].loop_sensor();
    }

    odometry();

    // Serial.print("TIME SINCE BEGIN : ");
    // Serial.println(timeSinceBegin / 1000.0 );
    goingHome = ((timeSinceBegin / 1000.0) > timeBeforeGoingHome) || (numberOfBottles >= numberMaxOfBottle);

    refreshAllPID(); /*
    Serial.print(currentAngle);
    Serial.print(" VS ");
    Serial.println(angleCompass);*/

    if (obstacleInFront() && !isCurrentlydodgingObstacle ) { //&& !goingToGetBottle

      //Serial.println("J'EFFECTUE l'EVITAGE D'OBSTACLE");
      dodgingObstacle();
    }

    Serial.print("Position du robot : (");
    Serial.print(positionOfRobot.x);
    Serial.print(";");
    Serial.print(positionOfRobot.y);
    Serial.print(") ");
    Serial.print("Current angle : ");
    Serial.print(currentAngle / PI * 180);
    Serial.println(";");
    if (
      !( //Don't look for the bottle if close to the rock area
        ((positionOfRobot.x > (sizeOfFullArena - sizeBadAreaGrassX - greyArea)) && (positionOfRobot.y < sizeBadAreaGrassY) && ((currentAngle < (PI / 2.0) ) || (currentAngle > (3.0 * PI / 2.0)))) ||
        ((positionOfRobot.x > (sizeOfFullArena - sizeBadAreaGrassX)) && (positionOfRobot.y < (sizeBadAreaGrassY + greyArea)) && (currentAngle > PI )) ||
        ((positionOfRobot.x > (sizeOfFullArena - sizeBadAreaGrassX - greyArea)) && (positionOfRobot.y < (sizeBadAreaGrassY + greyArea)) && (currentAngle > (3.0 * PI / 2.0))) ||

        //Don't look for the bottles if close to the grass area
        ((positionOfRobot.x < sizeBadAreaRockXY) && (positionOfRobot.y > (sizeOfFullArena - sizeBadAreaRockXY - greyArea)) && (currentAngle < PI)) ||
        ((positionOfRobot.x < (sizeBadAreaRockXY + greyArea)) && (positionOfRobot.y > (sizeOfFullArena - sizeBadAreaRockXY)) && (currentAngle > (PI / 2.0)) && (currentAngle < (3 * PI / 2))) ||
        ((positionOfRobot.x < (sizeBadAreaRockXY + greyArea)) && (positionOfRobot.y > (sizeOfFullArena - sizeBadAreaRockXY - greyArea)) &&  (currentAngle > (PI / 2.0)) && (currentAngle < (PI)))

      )

    ) {
      refreshAllPID();
      if (!isCurrentlydodgingObstacle && !goingHome) {
        // Serial.println("JE CHERChE DES BOUTEILLES");
        bottleDetection();
      }
    }
    else {
      time_ = 0;
      goingToGetBottle = false;
    }
    if (goingToGetBottle) {
      //         Serial.println("JE CHERCHE LA BOUTEILLE MIAM MIAM LA BOUTEILLE");
    }

    if (!travellingToADestination && !goingToGetBottle) {
      //Serial.println("JE CHOISIS UNE NOUVELLE DESTINATION");
      pickingADestination();
    }

    if (travellingToADestination && !goingToGetBottle) {
      //Serial.println("JE VAIS VERS LA DESTINATION");
      goingToALocation();
    }
  }

  if (count == maxIteration || robotIsHome ) {

    odometry();
    if (sensorsObstacle[0].get_value() > sizeHeight) { //Checks if it can turn to the left
      do {
        dodge_L(optimalSpeedTurn, optimalSpeedTurn);
        refreshAllPID();
        odometry();
        delay(30);
      } while (   currentAngle > (PI / 3.0) || currentAngle < (PI / 6.0));
    }
    else {//Otherwise turns to the right
      do {
        dodge_R(optimalSpeedTurn, optimalSpeedTurn);
        odometry();
        refreshAllPID();
        delay(30);
      } while (   currentAngle > (PI / 3.0) || currentAngle < (PI / 6.0));
    }

    while (!noObstacleBehind()) { //Makes sur the wall behind is far enought so that the back can open
      advance(speedForward * 0.2, speedForward * 0.2);
      odometry();
      refreshAllPID();
      delay(10);
    }

    count = maxIteration + 2;
    robotIsHome = false;
    stop();
    back();
  }



}

bool obstacleInFront() {
  for (int i = 0; i < numberOfSensorObstacle; i++) {
    sensorsObstacle[i].loop_sensor();
  }
  for (int i = 0; i < numberOfSensorObstacle; i++) {
    if (sensorsObstacle[i].get_value() < 75) {
      // Serial.print("VALUE OF THE BOOL : ");
      // Serial.println(isCurrentlydodgingObstacle);
      // Serial.println("OUI OUI OUI");
      //  Serial.println("OBSTACLE DETECTED");
      return true;
    }
  }
  //Serial.println("OUI OUI OUI");
  return false;
}

//Checks the back sensors to make sure the robot can go backward
//Returns false it there is an obstacle
bool noObstacleBehind() {
  for (int i = 0; i < numberOfSensorsBack; i++) {

    if (sensorsBack[i].get_value() < thresholdBackSensors) {
      return false;
    }
  }
  return true;
}

void refreshAllPID() {
  leftPID.Compute();
  rightPID.Compute();
  analogWrite(E1, pwmOutLeft);
  analogWrite(E2, pwmOutRight);
}
/*
  if(obstacleInFront()){
  return;
  }*/

void bottleDetection() {

  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].loop_sensor();
  }

  if (sensorsObstacle[0].get_value() > 1.5 * cutOffDistance) { //No obstacle detected --> T=0
    if (sensorsFront[1].get_value() > cutOffDistance) { //Nothing on middle sensor --> T=0 M=0
      if (sensorsFront[0].get_value() > cutOffDistance) { // Nothing on left sensor --> T=0 M=0 L=0
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=0 L=0 R=1
          goingToGetBottle = true;
          if (sensorsFront[2].get_value() < veryCloseDistance) { // Right sensor detects --> T=0 M=0 L=0 R=1 but it's closer than focal point
            odometry();

            advance(speedForward * 0.6, speedForward * 0.2); //Turns right fast
            for (int i = 0; i < 15; i++) {

              delay(10);
              odometry();
              refreshAllPID();
            }
          }
          else {
            odometry();
            advance(speedForward * 0.4, speedForward * 0.6); //Turns left slowly
            for (int i = 0; i < 15; i++) {
              delay(10);
              odometry();
              refreshAllPID();
            }
          }

        }
        else {
          goingToGetBottle = false;
          //Serial.println("YA RIEN SUR LES SENSORS MON POTE");
          //Do nothing ==> // Nothing detected --> T=0 M=0 L=0 R=0
        }
      }
      else { // Left sensor detects --> T=0 M=0 L=1
        goingToGetBottle = true;
        if (sensorsFront[2].get_value() > cutOffDistance) { // Nothing on right sensor --> T=0 M=0 L=1 R=0
          if (sensorsFront[0].get_value() < veryCloseDistance) { // Only left sensor detects --> T=0 M=0 L=1 R=0 but it's closer than focal point
            odometry();
            advance(speedForward * 0.2, speedForward * 0.6); //Turns left fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              odometry();
              refreshAllPID();
            }
          }
          else {
            odometry();
            advance(speedForward * 0.6, speedForward * 0.4); //Turns right slowly
            for (int i = 0; i < 15; i++) {
              delay(10);
              odometry();
              refreshAllPID();
            }
          }

        }
      }
    }
    else { //Middle sensor detects --> T=0 M=1
      goingToGetBottle = true;
      if (sensorsFront[0].get_value() < cutOffDistance) { // Left sensor detects --> T=0 M=1 L=1
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=1 L=1 R=1


          //Serial.println(currentAngle / PI * 180);
          odometry();
          advance(speedForward * 0.3, speedForward * 0.3); //Goes forward for a bit
          for (int i = 0; i < 100; i++) {
            refreshAllPID();
            delay(10);
            odometry();

          }
          //delay(1000);
          //Serial.println(currentAngle / PI * 180);

          odometry();
          stop();
          for (int i = 0; i < 15; i++) {
            delay(10);
            refreshAllPID();
            odometry();

          }
          //Serial.println(currentAngle / PI * 180);

          //Serial.println(currentAngle / PI * 180);
          arm();                                          //Grabs bottle
          odometry();
          goingToGetBottle = false;
          advance(speedForward * 0.5, speedForward * 0.5);
        }
        else { // Nothing on right detector --> T=0 M=1 L=1 R=0
          if (sensorsFront[0].get_value() < veryCloseDistance) { // Left sensor detects --> T=0 M=1 L=1 R=0 but it's closer than focal point
            odometry();
            advance(speedForward * 0.2, speedForward * 0.6); //Turns left fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              odometry();
              refreshAllPID();
            }
          }
          else {
            odometry();
            advance(speedForward * 0.6, speedForward * 0.2); //Turns right fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              odometry();
              refreshAllPID();
            }
          }

        }
      }
      else { // Nothing on left sensor --> T=0 M=1 L=0
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=1 L=0 R=1
          goingToGetBottle = true;
          if (sensorsFront[2].get_value() < veryCloseDistance) { // Right sensor detects --> T=0 M=1 L=0 R=1 but it's closer than focal point
            odometry();
            advance(speedForward * 0.6, speedForward * 0.2); //Turns right fast
            for (int i = 0; i < 15; i++) {
              delay(10);
              odometry();
              refreshAllPID();
            }
          }
          else {
            odometry();
            advance(speedForward * 0.2, speedForward * 0.6); //Turns left fast
            for (int i = 0; i < 15; i++) {
              delay(10);

              odometry();
              refreshAllPID();
            }
          }
        }
        else { // Nothing on Right sensor --> T=0 M=1 L=0 R=0
          odometry();
          advance(speedForward * 0.5, speedForward * 0.5); //Goes forward
          for (int i = 0; i < 15; i++) {
            delay(10);
            odometry();
            refreshAllPID();
          }
        }
      }
    }
  }
  else { // Top sensor detects --> T=1
    /*
      time_ = 0;
      isCurrentlydodgingObstacle  = true;
      goingBack = false;
      wasGoingBack = false;
      Serial.print("JE REPERE UN TRUC GENRE UN OBSTACLE A CETTE DISTANCE : ");
      Serial.println(sensorsFront[3].get_value());
      odometry();
      dodgingObstacle(sensorsFront[3].get_value());
      odometry();
      for (int i = 0; i < 15; i++) {
        delay(10);
        refreshAllPID();
      }*/
  }
}
//If a location hasn't been picked yet then it will chose one
void pickingADestination() {

  destinationAvailable = false;
  totalFar = 0;/*
      Serial.print("Position du robot : (");
      Serial.print(positionOfRobot.x);
      Serial.print(";");
      Serial.print(positionOfRobot.y);
      Serial.println(")");*/

  for (int i = 0; i < 3; i++) {
    possibilities[i].x = positionOfRobot.x + r * cos(angles[i] + currentAngle);
    possibilities[i].y = positionOfRobot.y + r * sin(angles[i] + currentAngle); /*
        Serial.print(possibilities[i].x);
        Serial.print( " , ");
        Serial.println(possibilities[i].y);*/
    possibilities[i].canGoThere = checkIfCanGo(possibilities[i]) ;//&& pinsObstacle[i].get_value<100;

    possibilities[i].howFar = returnPossibilitiesFromPosition( possibilities[i].x, possibilities[i].y , angles[i] );
    totalFar += possibilities[i].howFar;
    /*
      S

      Serial.print("Nombre de possibilité pour la position ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(possibilities[i].howFar);*/


    if (possibilities[i].canGoThere && (!wasGoingBack || i != 1)) {
      destinationAvailable = true;
      wasGoingBack = false;
    }
  }
  refreshAllPID();
  if (totalFar == 0)destinationAvailable = false;
  if (destinationAvailable) {
    travellingToADestination = true;
    if (!goingHome) {
      do {
        int random_ = random(0, totalFar);
        indexPosibility = fromProbaToIndex(possibilities[0].howFar, possibilities[1].howFar, possibilities[2].howFar, random_);
        // indexPosibility = random(0,3);
      } while (!possibilities[indexPosibility].canGoThere);


    }
    else {
      int indexToHome = 0;
      int distanceMin = 1131; //sqrt(2*800*800)
      int distanceI;
      if (positionOfRobot.x < 100 and positionOfRobot.y < 100) {
        robotIsHome = true;
      }
      for (int i = 0; i < 3; i++) {
        distanceI = sqrt(possibilities[i].x * possibilities[i].x + possibilities[i].y * possibilities[i].y);
        if (distanceI < distanceMin && possibilities[i].canGoThere) {
          indexToHome = i;
          distanceMin = distanceI;
        }
      }

      indexPosibility = indexToHome;
    }
  }
  else {
    goingBack = true;
    wasGoingBack = true;
    travellingToADestination = true;
  }
}


//If a location has been picked then the robot will do the pre-determined movement
void goingToALocation() {
  if (goingBack) {
    if (noObstacleBehind()) {
      if (time_ < 40) {
        back_off(optimalSpeedBackward, optimalSpeedBackward);
        //    Serial.println("GOING BACK");
        time_++;

      }

      else {
        travellingToADestination = false;
        time_ = 0;
        count++;
        goingBack = false;

      }
    }
    else {
      stop();
      travellingToADestination = false;
      time_ = 0;
      count++;
      goingBack = false;
    }
  }

  else {
    goingBack = false;
    if (indexPosibility == 0) {
      if (time_ < time_turn) {
        turn_L(optimalSpeedLower, optimalSpeedUpper);
        time_++;
        //Serial.println("MOVING LEFt");

      }
      else {
        time_ = 0;
        travellingToADestination = false;
        count++;
        isCurrentlydodgingObstacle  = false;

      }
    }
    if (indexPosibility == 1) {
      if (time_ < time_forward) {
        advance(optimalSpeedForward, optimalSpeedForward);
        time_++;
        //Serial.println("MOVING FORWARD");

      }
      else {
        time_ = 0;
        travellingToADestination = false;
        count++;
        isCurrentlydodgingObstacle  = false;

      }
    }
    if (indexPosibility == 2) {
      if (time_ < time_turn) {
        turn_R(optimalSpeedUpper, optimalSpeedLower);
        time_++;
        //Serial.println("MOVING RIGHT");

      }
      else {
        time_ = 0;
        travellingToADestination = false;
        count++;
        isCurrentlydodgingObstacle  = false;

      }
    }
    if (indexPosibility == 3) {
      if (time_ < time_dodge) {
        dodge_L(optimalSpeedTurn, optimalSpeedTurn);
        time_++;
        //Serial.println("DODGING LEFT");

      }
      else {
        time_ = 0;
        travellingToADestination = false;
        count++;
        isCurrentlydodgingObstacle  = false;
      }
    }
    if (indexPosibility == 4) {
      if (time_ < time_dodge) {
        dodge_R(optimalSpeedTurn, optimalSpeedTurn);
        time_++;
        //  Serial.println("DODGING RIGHT");

      }
      else {
        time_ = 0;
        travellingToADestination = false;
        count++;
        isCurrentlydodgingObstacle  = false;
      }

    }
  }
}
//Reading the angle from the compass, the angle read from the compass is going clock wise (counter trygonometric) and in degree. Both of these things have to be changed.
void readValueCompass() {
  char valeurByte[8];
  int stack = 0;
  boolean readByte = false;
  boolean value = false;
  double tempAngleCompass;
  Serial2.write(0x31); //Asking for the angle, for each command sent you get 8 byte as an answer
  //First byte, enter => New Line => hundreds of angle => tens of angle => bits of angle => Decimal point of angle => Decimal of angle => Calibrate sum
  while (!value) {

    if (Serial2.available()) {
      valeurByte[stack] = Serial2.read(); //Read the value & stacks it
      stack = (stack + 1) % 8; //Allows to read the full 8 bytes
      if (stack == 0) {
        tempAngleCompass = (valeurByte[2] - 48) * 100 + (valeurByte[3] - 48) * 10 + (valeurByte[4] - 48); //Computes the angle by reading bytes
        value = true;
      }
    }
  }
  //angleCompass = 2 * PI * (360 - tempAngleCompass) / 360 + initialDifference;
  valueFromCompass = checkBoundsRadian(toRadian(tempAngleCompass) - initialDifference);
  diff = checkBoundsDiffRadian(currentAngle - valueFromCompass); //En radian
  if (diff > diffMax) diffMax = diff;

  if (angleCompass > 2 * PI) {
    angleCompass -= 2 * PI;
  }
  else if (angleCompass < 0) {
    angleCompass += (2 * PI);
  }
  //currentAngle = valueFromCompass;
  /*
    Serial.print("THIS IS THE ANGLE FROM THE COMPASS VALUE : ");
    Serial.println(angleCompass / (2 * PI) * 360);
    Serial.print("THIS IS ANGLE FROM THE ODOMETRY : ");
    Serial.println(currentAngle / (2 * PI) * 360);
    Serial.print("THIS IS THE DIFF : ");
    Serial.println(abs(currentAngle - angleCompass) / (2 * PI) * 360);*/
  /*
    Serial.print(toDegree(currentAngle)); //En degré
    Serial.print(" ");
    Serial.print(toDegree(valueFromCompass )); //En degré
    Serial.print(" ");
    Serial.print(toDegree(diff));
    Serial.print(" ");
    Serial.println(toDegree(diffMax));*/
}

double toDegree(double value) {
  return checkBoundsDegree((value) / PI * 180);
}

double toRadian(double value) {
  return checkBoundsRadian((360 - value) / 180 * PI);
}

double checkBoundsDegree(double value) {
  if (value >= 360)return (value - 360);
  if (value < 0) return (value + 360);
  return value;
}

double checkBoundsRadian(double value) {
  if (value >= 2 * PI) return (value - (2 * PI));
  if (value < 0) return (value + (2 * PI));
  return value;
}

double checkBoundsDiffRadian(double value) {
  if (value >= PI)return 2 * PI - value;
  if (value < (-PI)) return 2 * PI - abs(value);
  if (value < 0.0) return abs(value);
  return value;
}

void dodgingObstacle() {
  time_ = 0;
  //isCurrentlydodgingObstacle  = true;
  goingBack = false;
  wasGoingBack = false;
  goingToGetBottle = false;
  /*
    const int thresholdDistance = 20;
    if (distanceToObstacle > thresholdDistance) {
      leftRight[0].x = positionOfRobot.x + (sizeBetweenWheels / 2) * cos(currentAngle + PI / 2);
      leftRight[0].y = positionOfRobot.y + (sizeBetweenWheels / 2) * sin(currentAngle + PI / 2);
      leftRight[0].canGoThere = checkIfCanGo(leftRight[0]);
      leftRight[1].x = positionOfRobot.x + (sizeBetweenWheels / 2) * cos(currentAngle + PI / 2);
      leftRight[1].y = positionOfRobot.y + (sizeBetweenWheels / 2) * sin(currentAngle + PI / 2);
      leftRight[1].canGoThere = checkIfCanGo(leftRight[1]);
      if (leftRight[0].canGoThere && leftRight[1].canGoThere ) {
        indexPosibility = random(3, 5);
      }
      else if (leftRight[0].canGoThere) {
        indexPosibility = 3;
      }
      else {
        indexPosibility = 4;
      }
      travellingToADestination = true;
    }
    else {
      if (noObstacleBehind()) {
        travellingToADestination = true;
        goingBack = true;
        wasGoingBack = true;
      }

    }*/
  double thresholdDistanceAvoid = 50;
  if (sensorsObstacle[0].get_value() < thresholdDistanceAvoid) { //S0=1
    if (sensorsObstacle[2].get_value() < thresholdDistanceAvoid) { //S2=1
      //Serial.println("S0 =1 S2 = 1");
      goingBack = true;
      travellingToADestination = true;
    }
    else { //S2=0
      //Serial.println("S0 =1 S2 = 0");
      isCurrentlydodgingObstacle  = true;
      indexPosibility = 4;
      time_ = time_dodge / 2.0;
      travellingToADestination = true;
    }
  }
  else { //S0=0
    if (sensorsObstacle[2].get_value() < thresholdDistanceAvoid) { //S2=1
      //Serial.println("S0 =0 S2 = 1");

      isCurrentlydodgingObstacle  = true;
      indexPosibility = 3;
      time_ = time_dodge / 2.0;
      travellingToADestination = true;
    }
    else { //S2=0
      if (sensorsObstacle[1].get_value() < thresholdDistanceAvoid) { //S1==1
        //Serial.println("S0 =0 S1 = 1 S2 = 0");

        goingBack = true;
        travellingToADestination = true;
      }
      else {
        return;
      }
    }
  }
  //  indexPosibility = random(3, 5);

}

int acc = 0;
//Computes the new position of the robot & the angle that has to be changed using the compass
void odometry() {
  previousTime = currentTime;
  currentTime = millis();
  diffTime = currentTime - previousTime;
  timeAfterDelay = millis();
  durationLeft = abs(leftEncoder.read()); //Reads the left accumulated encodeur
  durationRight = abs(rightEncoder.read()); //Reads the value accumulated on the right encodeur
  speedWheelLeft = 1000 * factorPulseToDistance * durationLeft / diffTime; //  cm/s
  speedWheelRight = 1000 * factorPulseToDistance * durationRight / diffTime; //  cm/s
  if (digitalRead(M1) == HIGH) {
    distanceLeft = factorPulseToDistance * durationLeft; //Distance travelled by the left wheel
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
  distanceCenter = (distanceLeft + distanceRight) / 2; //For the center of the two wheels

  phi = (distanceRight - distanceLeft) / sizeBetweenWheels;
  positionOfRobot.x = positionOfRobot.x + distanceCenter * cos(currentAngle);
  positionOfRobot.y = positionOfRobot.y + distanceCenter * sin(currentAngle);
  if (acc % 10 == 0) {
    //Updating the compass value
    // stop();
    // readValueCompass();
    acc = 1;
    //   currentAngle = angleCompass;
    // currentAngle = currentAngle + phi;
  }
  acc++;
  //  else {
  currentAngle = currentAngle + phi; //New angle for our robot, to calibrate with the compass
  //  }
  if (currentAngle > 2 * PI) {
    currentAngle -= 2 * PI;
  }
  else if (currentAngle < 0) {
    currentAngle += (2 * PI);
  }
  //currentAngle = angleCompass;
  leftEncoder.write(0); //Resets the accumulators to 0
  rightEncoder.write(0);
}

double distanceToBackcorners = sqrt(sizeBetweenWheels / 2 * sizeBetweenWheels / 2 + sizeHeight * sizeHeight); //The distance from the start

//Checking if a position is valid
bool checkIfCanGo(position_ destination) {
  position_ corners[4]; //Top left, top right, bottom right, bottom left
  corners[0].x = destination.x + cos(currentAngle + PI / 2) * sizeBetweenWheels / 2;
  corners[0].y = destination.y + sin(currentAngle + PI / 2) * sizeBetweenWheels / 2;

  corners[1].x = destination.x + cos(currentAngle - PI / 2) * sizeBetweenWheels / 2;
  corners[1].y = destination.y + sin(currentAngle - PI / 2) * sizeBetweenWheels / 2;

  corners[2].x = destination.x + cos(currentAngle - 3 * PI / 4) * distanceToBackcorners;
  corners[2].y = destination.y + sin(currentAngle - 3 * PI / 4) * distanceToBackcorners;

  corners[3].x = destination.x + cos(currentAngle + 3 * PI / 4) * distanceToBackcorners;
  corners[3].y = destination.y + sin(currentAngle + 3 * PI / 4) * distanceToBackcorners;
  /*
    for(int i=0; i<4;i++){
      Serial.print("THIS CORNER DOESNT WORK : ");
      Serial.print(corners[i].x);
      Serial.print(" ; ");
      Serial.print(corners[i].y);
      Serial.print(") THIS IS i :");
      Serial.println(i) ;
    }
    Serial.println("----------------");*/

  for (int i = 0; i < 4; i++) {
    if (!checkWalls(corners[i])) {
      return false;
    }
    //Uncomment this if u want to check full arena
    //!checkGrass(corners[i]) ||
    if ( !checkGrass(corners[i]) || !checkRocks(corners[i]) || !checkUpperPart(corners[i]) ) {
      return false;
    }
  }
  if (!checkWalls(destination)) {
    return false;
  }
  return true;
}

//Checks if a position is in a wall
bool checkWalls(position_ destination) {
  if (destination.x < epsilon || destination.x > (sizeOfFullArena - epsilon) || destination.y < epsilon || destination.y > (sizeOfFullArena - epsilon)) { //Don't get out of the arena
    return false;
  }
  return true; //Otherwise it is OK
}


//Check if a position is in the grass area
bool checkGrass(position_ destination) {

  if (goingHome) return true;
  if (destination.x > (sizeOfFullArena - sizeBadAreaGrassX - epsilon) && destination.y < (sizeBadAreaGrassY + epsilon)) { //Grass area
    return false;
  }
  return true;
}

//Check if a position is in the rock area
bool checkRocks(position_ destination) {
  if (destination.x < (sizeBadAreaRockXY + epsilon)  && destination.y > (sizeOfFullArena - sizeBadAreaRockXY - epsilon)) { //Rock area
    return false;
  }
  return true;
}


//Checks if a position is the upper part
bool checkUpperPart(position_ destination) {
  if (destination.x > (sizeOfFullArena - sizeBadAreaUpperX - epsilon) && destination.y > (sizeOfFullArena - sizeBadAreaUpperY - epsilon)) { //Upper ramp area
    return false;
  }
  return true;
}


//Returns the number of possible destinations available from a position
int returnPossibilitiesFromPosition(double x, double y, double angleToAdd) {
  int toReturn = 0;
  position_ arrayPossibilities[3];
  for (int i = 0; i < 3; i++) {
    arrayPossibilities[i].x = x + r * cos(angles[i] + currentAngle + angleToAdd);
    arrayPossibilities[i].y = y + r * sin(angles[i] + currentAngle + angleToAdd);
    arrayPossibilities[i].canGoThere = checkIfCanGo(arrayPossibilities[i]);
    if (arrayPossibilities[i].canGoThere) {
      toReturn++;
    }

  }
  return toReturn;
}

int fromProbaToIndex(int first, int second, int third, int randomNumber) {
  if (randomNumber < first)return 0;
  if (first <= randomNumber && randomNumber < (second + first))return 1;
  if ((second + first) <= randomNumber && randomNumber < (third + second + first)) return 2;
}

//The arm is going down
void arm() {
  numberOfBottles++;
  stop();
  servoArm.attach(pinservoArm);
  refreshAllPID();
  //Serial.println("Arm Turning...");
  for (int position = uplim; position < lowlim; position++) {
    servoArm.writeMicroseconds(position);
    refreshAllPID();
    delay(downspeed);
  }
  delay(waitingOnBottleTime);
  refreshAllPID();
  for (int position = lowlim; position > intermediatePosition; position--) {
    servoArm.writeMicroseconds(position);
    delay(upspeedFirstPart);
    refreshAllPID();
  }
  for (int position = intermediatePosition; position > uplim; position --) {
    servoArm.writeMicroseconds(position);
    delay(upspeedSecondPart);
    refreshAllPID();
  }
  servoArm.detach();
  pwmOutLeft = 0;
  pwmOutRight = 0;
  //Serial.println("-DONE TURNING-");
}

//The back is opening
void back() {
  delay(1000);
  stop();
  //Serial.println("Back opening...");
  for (int position = uplim_b; position > lowlim_b; position--) {
    servoBack.write(position);
    delay(1);
  }
  delay(waitingBottleOut);
  for (int position = lowlim_b; position < uplim_b; position++) {
    servoBack.write(position);
    delay(1);
  }
  //Serial.println("-DONE OPENING/CLOSING-");


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
  //leftPID.Compute();




}

//--------------------------------------BACK OFF

void back_off (char a, char b) //Move backward
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);

  targetSpeedLeft = a;
  targetSpeedRight = b;



}

//--------------------------------------TURN LEFT

void turn_L (char a, char b) //Turn Left
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  targetSpeedLeft = a;
  targetSpeedRight = b;

}


//--------------------------------------TURN RIGHT

void turn_R (char a, char b) //Turn Right
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  targetSpeedLeft = a;
  targetSpeedRight = b;

}
//------------------------------------------ DODGE RIGHT
void dodge_R (char a, char b) //Turn Right
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);

  targetSpeedLeft = a;
  targetSpeedRight = b;

}



//------------------------------------------DODGE LEFT
void dodge_L (char a, char b) //Turn Right
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);

  targetSpeedLeft = a;
  targetSpeedRight = b;

}
void recalibrate()
{

  double backToWheels = 35.5;
  // go to angle=0
  do {
    turn_R(0.3 * speedForward, 0.3 * speedForward);
    delay(10);
    odometry();
    refreshAllPID;
  } while (currentAngle > PI / 24 || currentAngle < 47 * PI / 24);

  // back off until you touch the wall
  do {
    back_off(speedForward * 0.3 * min(min(500, sensorsBack[1].get_value()) / min(500, sensorsBack[0].get_value()), 1.5), speedForward * 0.3 * min(min(500, sensorsBack[0].get_value()) / min(500, sensorsBack[1].get_value()), 1.5));
    delay(10);
    odometry();
    refreshAllPID();
  } while (sensorsBack[0].get_value() > 5 && sensorsBack[1].get_value() > 5);
  delay(1000);

  
  
  // calibrate the x position and the angle
  currentAngle = 0;
  positionOfRobot.x = backToWheels;

  // go forward a bit
  advance(speedForward * 0.3, speedForward * 0.3); //Goes forward for a bit
  for (int i = 0; i < 100; i++) {
    refreshAllPID();
    delay(10);
    odometry();
  }

  // go to angle=PI/2
  do {
    turn_L(0.3 * speedForward, 0.3 * speedForward);
    delay(10);
    odometry();
    refreshAllPID;
  } while (currentAngle < 11 * PI / 24 || currentAngle > 13 * PI / 24);


  // back off until you touch the wall
  do {
    back_off(speedForward * 0.3 * min(min(500, sensorsBack[1].get_value()) / min(500, sensorsBack[0].get_value()), 1.5), speedForward * 0.3 * min(min(500, sensorsBack[0].get_value()) / min(500, sensorsBack[1].get_value()), 1.5));
    delay(10);
    odometry();
    refreshAllPID();
  } while (sensorsBack[0].get_value() > 5 && sensorsBack[1].get_value() > 5);
  delay(1000);
  
  // recalibrate y position and the angle
  currentAngle = PI / 2;
  positionOfRobot.y = backToWheels;

  Serial.println(positionOfRobot.x);
}
