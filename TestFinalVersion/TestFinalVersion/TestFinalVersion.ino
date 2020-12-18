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

const int sizeOfFullArena = 800; //IF FULL ARENA

double epsilon = 20; //How close you dont want to get close to the area you don't want to go in
const double r = 40; //Radius of circle
position_ positionOfRobot; //Our robot


position_ leftRight[2]; //Possibilities for dodging the obsacle


position_ possibilities[3]; //Posibilities of where to go
const double angles[3] = {PI/4, 0,-PI/4,};


bool destinationAvailable=false; //If there is somewhere to go
boolean travellingToADestination = false; //If it is going somewhere
int indexPosibility;
int count= 0;
const int maxIteration = 200;
bool robotIsHome = false;
bool goingBack = false; //If the robot is moving backward
bool wasGoingBack = false; //If the last movement was to go backward
bool goingHome = false; //If the robot is going home
bool isCurrentlydodgingObstacle = false; //If the robot is currently dodging an obstacle
double ratioBeforeGoingHome = 0.75 ;
int totalFar = 0;
double valueX[4*maxIteration];
double valueY[4*maxIteration];


//Odometry 
double speedWheelRight = 0; //cm/s Speed of left wheel
double speedWheelLeft = 0; //Speed of right wheel
double sizeBetweenWheels = 38.3; //cm
double sizeHeight = 41; //cm
double timeBetweenRead = 1;
double distanceLeft ;
double distanceRight;
double distanceCenter = (distanceLeft + distanceRight)/2;
double phi = 0;
double currentAngle = PI/4; //Starting angle


//All the speed for going forward/going back/left & right
const int optimalSpeedLower = (r-sizeBetweenWheels/2)*(PI/4);
const int optimalSpeedUpper = (r+sizeBetweenWheels/2)*(PI/4);
const int optimalSpeedForward = r;
const int optimalSpeedBackward = optimalSpeedForward;
const int optimalSpeedTurn = sizeBetweenWheels * (PI/2);


/////////Number of iteration for each movement (Each iteration is about X ms, so if time_ is 40 then the robot will do the move for 40*X ms/)
int time_ = 0;
const int time_forward = 40;
const int time_turn = 40;
const int time_dodge = 10;

/////////////////////////////////////////////////////
byte incomingByte; //Byte being read from user
//All the speeds are in ms/angle (it is not a speed I know it's the inverse of a speed

////////////////Compass//////////
double angleCompass = 0;
double initialDifference = 0;

//////////////////////SENSORS//////////////////////////////////////

const int window_size = 5;

#define model80 1080
#define model30 430
#define pin A13

struct sensor{ 
  double read_value = 0;
  double values_sensor[window_size];
  int index = 0;
  double average = 0;
  int pinSensor;
  SharpIR* mySensor;
  
  void sensorInit(int whichPin, int maxDistance){ //Max distance for the model, 30 if 30 cm max models, 80 otherwise
      pinSensor = whichPin;
      if(maxDistance ==30){
        mySensor = new SharpIR(pinSensor, model30);
      }
      else if(maxDistance == 80){
         mySensor = new SharpIR(pinSensor, model80);
      }
      else{
         mySensor = new SharpIR(pinSensor, model30);
      }
        
  }
  void loop_sensor(){
    read_value =  mySensor->distance(); 
    values_sensor[index] = read_value;
    index++;
    index = index%window_size;
  }

  double get_value(){
    average = 0;
    for(int i=0; i<window_size;i++){
      average += values_sensor[i];
    }
    return average/window_size;  
  }
};
const int numberOfSensorsBack = 2;
const int numberOfSensorObstacle = 1;


sensor sensorsBack[numberOfSensorsBack];
sensor sensorsObstacle[numberOfSensorObstacle];

int pinsBack[] = {12,13}; //Analog Pins for the back sensors

int pinsFront[] = {4}; //Analog Pins for the back sensors

double thresholdBackSensors = 15; //The value in cm before the robot stops going backward if there is an obstacle at less than this distance.


///////Servo of the Arm////////////

Servo servoArm;
int pinservoArm =  8; //Pin of servo for the ARM (PWM)
int positionOfArm;
int uplim = 500; //Position when the arm is at the top
int lowlim =2700; //Position when the arm is at the bottom
int upspeedFirstPart = 3;
int upspeedSecondPart = 3;
int downspeed =1;
int setspeed = 10;
int waitingOnBottleTime = 1000; //Time waited on the bottle
int intermediatePosition = 1000;


///////Servo of the back/////////
Servo servoBack;
int pinServoBack = 9; //Pin of servo for the back (PWM)
int positionOfBack;
int uplim_b = 70; //Position of back when close
int lowlim_b = 0; //Position of back when open
int speedBack = 50; //Speed back is opening/closing
int waitingBottleOut = 3000; //Waiting for bottle to go out

//Motors, M1 is left wheel, M2 is right wheel
int E1 = 6;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)
int E2 = 7; //M2 Speed Control (PWM)
int M2 = 29; //M2 Direction control (Digital)
int speedForward = 30 ; //Speed moving forward cm/s
int speedBackward = 50; //Speed moving backward cm/s
int speedTurning = 30; //Speed while turning cm/s
double diameterWheels = 12; //cm
double gearRatio = 74.83; //gear ratio of our pololu;
double countsPerRevolution = 48;
const double factorPulseToDistance = PI*diameterWheels/(countsPerRevolution*gearRatio);

//Motors Encodeurs


//Left wheel
const byte leftEncoder0pinA =  3;//Pin for left motor Encodeur  (must be a pin to use interrupt)
const byte leftEncoder0pinB = 4;//Second pin
double durationLeft = 0;
Encoder leftEncoder(leftEncoder0pinA,leftEncoder0pinB);

//Right wheel
const byte rightEncoder0pinA =  2;//first Pin for right motor Encodeur (must be a pin to use interrupt)
const byte rightEncoder0pinB = 5;//Second pin
double durationRight = 0;
Encoder rightEncoder(rightEncoder0pinA,rightEncoder0pinB);

//Time between each loop
unsigned long previousTime = millis();
unsigned long currentTime = millis();
unsigned long diffTime = currentTime - previousTime;
unsigned long timeBeforeDelay = millis();
unsigned long timeAfterDelay = millis();


//PID for motor control

//For the left motor
double targetSpeedLeft = 50; //
double pwmOutLeft = 0;
PID leftPID(&speedWheelLeft, &pwmOutLeft, &targetSpeedLeft,5.1,0.5,0.005, DIRECT); 

//For the right motor
double targetSpeedRight = 0;
double pwmOutRight = 0;
PID rightPID(&speedWheelRight, &pwmOutRight, &targetSpeedRight,5.1,0.5,0.005, DIRECT); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);      //Set Baud Rate    
  Serial3.begin(9600); //Compass has a Baud Rate of 9600
  randomSeed(analogRead(13));

 // readValueCompass();
  initialDifference = currentAngle - angleCompass ;
  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M2, OUTPUT);
  positionOfRobot.x = 50;
  positionOfRobot.y = 50;
  //setting up servos
  servoArm.attach(pinservoArm);
  servoBack.attach(pinServoBack);
  positionOfArm = servoArm.read(); 
  Serial.println("Reseting the arm...");
  servoArm.writeMicroseconds(500);
  Serial.println("DONE");
  positionOfBack = servoBack.read(); 
  Serial.println("Reseting the back...");
  servoBack.write(70);

  
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
  for(int i=0; i<numberOfSensorsBack;i++){
    sensorsBack[i].sensorInit(pinsBack[i], 30);
  }
  for(int i=0; i<numberOfSensorObstacle;i++){
    sensorsObstacle[i].sensorInit(pinsFront[i], 80);
  }

}

double compassArray[5];


void loop() {
  
    if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'x'://Move Forward
        stop();
        count = 100;
        break;
      }
    }
    else stop();
  }
  if(count<maxIteration && !robotIsHome){
    Serial.print("This is COUNT :");
    Serial.println(count);
    if(time_==0){
      epsilon+=0.2;
      
    }
    //UPDATING THE BACK SENSORS :
    for(int i=0;i<1;i++){
      sensorsBack[i].loop_sensor();
    }
    for(int i=0; i<numberOfSensorObstacle;i++){
      sensorsObstacle[i].loop_sensor();
    } 
    previousTime = currentTime;
    currentTime = millis();
    diffTime = currentTime - previousTime;
    Serial.print("THIS IS DIFF TIME : ");
    Serial.println(diffTime);
  //  delay(50);
    timeBeforeDelay = millis();
    //Serial.print("THIS IS MINUS : ");
   // Serial.println(20 - (timeBeforeDelay -  timeAfterDelay));
  //  delay(30 - (timeBeforeDelay -  timeAfterDelay));//-(timeBeforeDelay -  timeAfterDelay)); //Always 20 ms

    timeAfterDelay = millis();
    durationLeft = abs(leftEncoder.read()); //Reads the left accumulated encodeur
    durationRight = abs(rightEncoder.read()); //Reads the value accumulated on the right encodeur
    speedWheelLeft = 1000*factorPulseToDistance*durationLeft/diffTime; //  cm/s
    speedWheelRight = 1000*factorPulseToDistance*durationRight/diffTime;//  cm/s
    odometry();
   /* if(count%10==0 and time_==0){
      //Updating the compass value
      stop();
      readValueCompass();
      currentAngle = angleCompass;
    }*/
    leftEncoder.write(0); //Resets the accumulators to 0
    rightEncoder.write(0);
    goingHome = count>maxIteration*ratioBeforeGoingHome;
    leftPID.Compute();
    rightPID.Compute();
    analogWrite(E1, pwmOutLeft);
    analogWrite(E2, pwmOutRight);/*
    Serial.print(currentAngle);
    Serial.print(" VS ");
    Serial.println(angleCompass);*/
    
          Serial.print("Position du robot : (");
      Serial.print(positionOfRobot.x);
      Serial.print(";");
      Serial.print(positionOfRobot.y);
      Serial.println(")");
    /*
    
    Serial.print("Diff on X : ");
    Serial.print(possibilities[indexPosibility].x - positionOfRobot.x);
    Serial.print("Diff on Y : ");
    Serial.print(" ");
    Serial.println(possibilities[indexPosibility].y - positionOfRobot.y);*/

    if(obstacleInFront() && !isCurrentlydodgingObstacle ){
      time_ = 0;

      isCurrentlydodgingObstacle  = true;
      goingBack = false;
      wasGoingBack = false;
      dodgingObstacle(sensorsObstacle[0].get_value());
    }
    if(!travellingToADestination){
      pickingADestination();  
    }
    
    else if(travellingToADestination){
      goingToALocation();
    }
  }
   if(count==maxIteration || robotIsHome ){
    
    Serial.println("DONE!");
    Serial.print("x = [");
    for(int i=0;i<4*(maxIteration-1);i++){
      Serial.print(valueX[i]);
      Serial.print(",");
    }
    Serial.print(valueX[4*maxIteration-3]);
    Serial.print(",");
    Serial.print(valueX[4*maxIteration-2]);
    Serial.print(",");
    Serial.print(valueX[4*maxIteration-1]);  
    Serial.println("]");
    Serial.print("y = [");
    for(int i=0;i<4*(maxIteration-1);i++){
      Serial.print(valueY[i]);
      Serial.print(",");
    }
    Serial.print(valueY[4*maxIteration-3]);
    Serial.print(",");
    Serial.print(valueY[4*maxIteration-2]);
    Serial.print(",");
    Serial.print(valueY[4*maxIteration-1]);
    Serial.println("]");
    count= maxIteration+2;
    robotIsHome=false; 
    stop();
    
  }
  
            

}

bool obstacleInFront(){
    for(int i=0; i<numberOfSensorObstacle;i++){
      if(sensorsObstacle[i].get_value()<80){
        Serial.print("VALUE OF THE BOOL : ");
        Serial.println(isCurrentlydodgingObstacle);
        Serial.println("OBSTACLE DETECTED");
        return true;
      }
    }     
    return false;
}

//Checks the back sensors to make sure the robot can go backward
//Returns false it there is an obstacle
bool noObstacleBehind(){
  for(int i=0; i<numberOfSensorsBack;i++){
    if(sensorsBack[i].get_value()<thresholdBackSensors){
      return false;
    }
  }
  return true;
}


//If a location hasn't been picked yet then it will chose one
void pickingADestination(){

      destinationAvailable=false;
      totalFar = 0;/*
      Serial.print("Position du robot : (");
      Serial.print(positionOfRobot.x);
      Serial.print(";");
      Serial.print(positionOfRobot.y);
      Serial.println(")");*/
       
      for(int i=0;i<3;i++){
        possibilities[i].x = positionOfRobot.x + r*cos(angles[i]+currentAngle);
        possibilities[i].y = positionOfRobot.y + r*sin(angles[i]+currentAngle); /*
        Serial.print(possibilities[i].x);
        Serial.print( " , ");
        Serial.println(possibilities[i].y);*/
        possibilities[i].canGoThere = checkIfCanGo(possibilities[i]);
        possibilities[i].howFar = returnPossibilitiesFromPosition( possibilities[i].x,possibilities[i].y ,angles[i] );
        totalFar+=possibilities[i].howFar;
        /*
        S
        
        Serial.print("Nombre de possibilité pour la position ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(possibilities[i].howFar);*/


        if(possibilities[i].canGoThere && (!wasGoingBack || i!=1)){
          destinationAvailable=true;
          wasGoingBack = false;
        }
      }
      if(totalFar==0)destinationAvailable=false;
      if(destinationAvailable){
        travellingToADestination = true;
        if(!goingHome){
          do{
            int random_ = random(0,totalFar);
            indexPosibility = fromProbaToIndex(possibilities[0].howFar,possibilities[1].howFar,possibilities[2].howFar,random_);
           // indexPosibility = random(0,3);
          }while(!possibilities[indexPosibility].canGoThere);

                    
        }
        else{
          int indexToHome = 0;
          int distanceMin = 1131; //sqrt(2*800*800)
          int distanceI;
          if(positionOfRobot.x<100 and positionOfRobot.y < 100){
            robotIsHome = true;
          }
          for(int i= 0; i<3;i++){
            distanceI = sqrt(possibilities[i].x * possibilities[i].x + possibilities[i].y*possibilities[i].y);
            if(distanceI<distanceMin && possibilities[i].canGoThere){
              indexToHome = i;
              distanceMin = distanceI;
            }
          }

          indexPosibility = indexToHome;
        }
      }
      else{
        goingBack = true;
        wasGoingBack = true;
        travellingToADestination = true;
      }
}


//If a location has been picked then the robot will do the pre-determined movement
void goingToALocation(){
      if(goingBack){
        if(!noObstacleBehind()){
          if(time_<40){
            back_off(optimalSpeedBackward,optimalSpeedBackward);
         //    Serial.println("GOING BACK");
             time_++;
  
          }
        
          else{
            travellingToADestination = false;  
            time_ = 0;
            count++; 
            goingBack = false; 
  
          }
        }
        else{
          stop();
          travellingToADestination = false;  
          time_ = 0;
          count++; 
          goingBack = false; 
        }
     }

      else{
        goingBack = false;
        if(indexPosibility==0){
          if(time_<time_turn){
            turn_L(optimalSpeedLower, optimalSpeedUpper);  
            time_++;         
          }
          else{
            time_ =0;
            travellingToADestination = false;
            count++; 
            isCurrentlydodgingObstacle  = false;

          }
               Serial.println("MOVING LEFt");
        }
        if(indexPosibility==1){
          if(time_<time_forward){
            advance(optimalSpeedForward,optimalSpeedForward);
            time_++; 
          }
          else{
            time_ =0;
            travellingToADestination = false;
            count++; 
            isCurrentlydodgingObstacle  = false;

          }
               Serial.println("MOVING FORWARD");
        }
        if(indexPosibility==2){
          if(time_<time_turn){
            turn_R(optimalSpeedUpper,optimalSpeedLower);
            time_++; 
          }
          else{
            time_ =0;
            travellingToADestination = false;
            count++; 
            isCurrentlydodgingObstacle  = false;

          }
                Serial.println("MOVING RIGHT");
        }
        if(indexPosibility==3){
          if(time_<time_dodge){
           dodge_L(optimalSpeedTurn, optimalSpeedTurn);
           time_++; 
          }
          else{
            time_ =0;
            travellingToADestination = false;
            count++; 
            isCurrentlydodgingObstacle  = false;
          }
                Serial.println("DODGING LEFT");
        }
        if(indexPosibility==4){
          if(time_<time_dodge){
           dodge_R(optimalSpeedTurn, optimalSpeedTurn);
           time_++; 
          }
           else{
            time_ =0;
            travellingToADestination = false;
            count++; 
            isCurrentlydodgingObstacle  = false;
          }

                Serial.println("DODGING RIGHT");
        }                               
  }
}
  
//Reading the angle from the compass, the angle read from the compass is going clock wise (counter trygonometric) and in degree. Both of these things have to be changed.
void readValueCompass(){
  char valeurByte[8];
  int stack =0;
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
  angleCompass = 2*PI*(360-tempAngleCompass)/360 + initialDifference;   
  if (angleCompass>2*PI){
    angleCompass-=2*PI;
  }
  else if (angleCompass<0){ 
    angleCompass+=(2*PI);
  }
        Serial.print("THIS IS THE ANGLE FROM THE COMPASS VALUE : ");
    Serial.println(angleCompass/(2*PI)*360);
    Serial.print("THIS IS ANGLE FROM THE ODOMETRY : ");
    Serial.println(currentAngle/(2*PI)*360);
        Serial.print("THIS IS THE DIFF : ");
    Serial.println(abs(currentAngle - angleCompass)/(2*PI)*360);
}



void dodgingObstacle(double distanceToObstacle){
  const int thresholdDistance = 20;
  if(distanceToObstacle>thresholdDistance){
    leftRight[0].x = positionOfRobot.x + (sizeBetweenWheels/2)*cos(currentAngle+PI/2); 
    leftRight[0].y = positionOfRobot.y + (sizeBetweenWheels/2)*sin(currentAngle+PI/2); 
    leftRight[0].canGoThere = checkIfCanGo(leftRight[0]);
    leftRight[1].x = positionOfRobot.x + (sizeBetweenWheels/2)*cos(currentAngle+PI/2); 
    leftRight[1].y = positionOfRobot.y + (sizeBetweenWheels/2)*sin(currentAngle+PI/2); 
    leftRight[1].canGoThere = checkIfCanGo(leftRight[1]);
    if(leftRight[0].canGoThere && leftRight[1].canGoThere ){
      indexPosibility = random(3,5);
    }
    else if(leftRight[0].canGoThere){
      indexPosibility = 3;
    }
    else{
      indexPosibility = 4;
    }
    travellingToADestination = true;
  }
  else{
    if(noObstacleBehind()){
      travellingToADestination = true;
      goingBack = true;
      wasGoingBack = true;
    }
    
  }
}

//Computes the new position of the robot & the angle that has to be changed using the compass
void odometry(){
  if(digitalRead(M1) == HIGH){
    distanceLeft = factorPulseToDistance*durationLeft; //Distance travelled by the left wheel   
  }
  else{
    distanceLeft = -factorPulseToDistance*durationLeft; //Distance travelled by the left wheel   
  }
  if(digitalRead(M2) == LOW){
    distanceRight = factorPulseToDistance*durationRight; //Distance travelled by the right wheel   
  }
  else{
    distanceRight = -factorPulseToDistance*durationRight; //Distance travelled by the right wheel   
  } 
  distanceCenter = (distanceLeft + distanceRight)/2; //For the center of the two wheels

  phi = (distanceRight - distanceLeft)/sizeBetweenWheels;
  positionOfRobot.x = positionOfRobot.x + distanceCenter*cos(currentAngle);
  positionOfRobot.y = positionOfRobot.y + distanceCenter*sin(currentAngle);
  currentAngle = currentAngle + phi; //New angle for our robot, to calibrate with the compass
  if (currentAngle>2*PI){
    currentAngle-=2*PI;
  }
  else if (currentAngle<0){
    currentAngle+=(2*PI);
  }
//currentAngle = angleCompass;
}

double distanceToBackcorners = sqrt(sizeBetweenWheels/2 * sizeBetweenWheels/2 + sizeHeight * sizeHeight); //The distance from the start

//Checking if a position is valid
bool checkIfCanGo(position_ destination){
  position_ corners[4]; //Top left, top right, bottom right, bottom left
  corners[0].x = destination.x + cos(currentAngle + PI/2)*sizeBetweenWheels/2;
  corners[0].y = destination.y + sin(currentAngle + PI/2)*sizeBetweenWheels/2;
  
  corners[1].x = destination.x + cos(currentAngle - PI/2)*sizeBetweenWheels/2;
  corners[1].y = destination.y + sin(currentAngle - PI/2)*sizeBetweenWheels/2;
  
  corners[2].x = destination.x + cos(currentAngle - 3*PI/4)*distanceToBackcorners;
  corners[2].y = destination.y + sin(currentAngle - 3*PI/4)*distanceToBackcorners;
  
  corners[3].x = destination.x + cos(currentAngle + 3*PI/4)*distanceToBackcorners;
  corners[3].y = destination.y + sin(currentAngle + 3*PI/4)*distanceToBackcorners;
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

  for(int i=0; i<4;i++){
    if(!checkWalls(corners[i])){
      return false;
    }
    //Uncomment this if u want to check full arena
    //!checkGrass(corners[i]) || 
    if( !checkRocks(corners[i]) || !checkUpperPart(corners[i]) ){
      return false;
    }
  }
  if(!checkWalls(destination)){
    return false;
  }
  return true;
}

//Checks if a position is in a wall
bool checkWalls(position_ destination){
  if(destination.x<epsilon || destination.x>(sizeOfFullArena-epsilon) || destination.y<epsilon || destination.y>(sizeOfFullArena-epsilon)){ //Don't get out of the arena
    return false;
  }
  return true; //Otherwise it is OK
}


//Check if a position is in the grass area
bool checkGrass(position_ destination){
  if(destination.x>(sizeOfFullArena-sizeBadAreaGrassX-epsilon) && destination.y<(sizeBadAreaGrassY+epsilon)){ //Grass area
    return false;
  }
  return true;
}

//Check if a position is in the rock area
bool checkRocks(position_ destination){
  if(destination.x<(sizeBadAreaRockXY+epsilon)  && destination.y>(sizeOfFullArena-sizeBadAreaRockXY-epsilon)){ //Rock area
    return false;
  }
  return true;
}


//Checks if a position is the upper part
bool checkUpperPart(position_ destination){
  if(destination.x>(sizeOfFullArena-sizeBadAreaUpperX-epsilon) && destination.y>(sizeOfFullArena-sizeBadAreaUpperY-epsilon)){ //Upper ramp area
    return false;
  }
  return true;
}


//Returns the number of possible destinations available from a position
int returnPossibilitiesFromPosition(double x, double y, double angleToAdd){
  int toReturn = 0;
  position_ arrayPossibilities[3];
  for(int i=0;i<3;i++){
    arrayPossibilities[i].x = x + r*cos(angles[i]+currentAngle+angleToAdd);
    arrayPossibilities[i].y = y + r*sin(angles[i]+currentAngle+angleToAdd); 
    arrayPossibilities[i].canGoThere = checkIfCanGo(arrayPossibilities[i]);
    if(arrayPossibilities[i].canGoThere){
      toReturn++;
    }

  }
  return toReturn;
}

int fromProbaToIndex(int first, int second, int third, int randomNumber){
  if(randomNumber<first)return 0;
  if(first<=randomNumber && randomNumber<(second+first))return 1;
  if((second+first)<=randomNumber && randomNumber<(third+second+first)) return 2;
}

//The arm is going down
void arm(){
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
   for (int position = intermediatePosition; position > uplim; position-=2) {
     servoArm.writeMicroseconds(position);
     delay(upspeedSecondPart);
   }

   Serial.println("-DONE TURNING-");
}

//The back is opening
void back(){
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
  digitalWrite(M1,LOW);
  digitalWrite(M2,LOW);
  targetSpeedLeft = 0;
  targetSpeedRight = 0;
  digitalWrite(E1,0);
  digitalWrite(E2,0);
}

//--------------------------------------ADVANCE

void advance(char a, char b)  //Move forward
{

  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  
  targetSpeedLeft = a;
  targetSpeedRight = b;
  //leftPID.Compute();

     
  

}

//--------------------------------------BACK OFF

void back_off (char a, char b) //Move backward
{
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  
  targetSpeedLeft = a;
  targetSpeedRight = b;
  

  
}

//--------------------------------------TURN LEFT

void turn_L (char a,char b)  //Turn Left
{
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  
  targetSpeedLeft = a;
  targetSpeedRight = b;
  
}


//--------------------------------------TURN RIGHT

void turn_R (char a,char b)  //Turn Right
{
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  
  targetSpeedLeft = a;
  targetSpeedRight = b;
   
}
//------------------------------------------ DODGE RIGHT
void dodge_R (char a,char b)  //Turn Right
{
  digitalWrite(M1,HIGH);
  digitalWrite(M2,HIGH);
  
  targetSpeedLeft = a;
  targetSpeedRight = b;
   
}



//------------------------------------------DODGE LEFT
void dodge_L (char a,char b)  //Turn Right
{
  digitalWrite(M1,LOW);
  digitalWrite(M2,LOW);
  
  targetSpeedLeft = a;
  targetSpeedRight = b;
   
}
