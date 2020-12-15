
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
const int sizeBadAreaUpperX = 200;
const int sizeBadAreaUpperY = 300;


const int sizeBadArea = 300; //3 meters for each arena we don't want to go in
const int sizeOfFullArena = 800;
const double epsilon = 10; //How close you dont want to get close to the area you don't want to go in
const double r = 40; //Radius of circle

position_ leftRight[2];

position_ possibilities[3]; //Posibilities of where to go
const double angles[3] = {PI/4, 0,-PI/4,};
position_ positionOfRobot; //Our robot
double currentAngle = PI/4; //Starting angle
bool destinationAvailable=false; //If there is somewhere to go
boolean travellingToADestination = false; //If it is going somewhere
int indexPosibility;
int count= 0;
const int maxIteration = 100;
bool goingBack = false; //If the robot is moving backward
bool wasGoingBack = false; //If the last movement was to go backward
bool goingHome = false; //If the robot is going homer
double ratioBeforeGoingHome = 0.85;
int totalFar = 0;
double valueX[4*maxIteration];
double valueY[4*maxIteration];
int time_ = 0;
//Odometry 69*PI/16; 31*PI/16;
double speedWheelRight = 0; //cm/s Speed of left wheel
double speedWheelLeft = 0; //Speed of right wheel
double sizeBetweenWheels = 38; //cm
double timeBetweenRead = 1;
double distanceLeft = speedWheelLeft * timeBetweenRead/100;
double distanceRight = speedWheelRight * timeBetweenRead/100;
double distanceCenter = (distanceLeft + distanceRight)/2;
double phi = 0;
bool backward;
const int optimalSpeedLower = (r-sizeBetweenWheels/2)*(PI/4)/4;
const int optimalSpeedUpper = (r+sizeBetweenWheels/2)*(PI/4)/4;
const int optimalSpeedWheelTurn = (sizeBetweenWheels) * (PI/2);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  randomSeed(3543);
  positionOfRobot.x = 50;
  positionOfRobot.y = 50;
}

void odometry(){
  distanceLeft = speedWheelLeft *  timeBetweenRead/10; //Distance from the left wheel since last check
  distanceRight = speedWheelRight *  timeBetweenRead/10; //for the right wheel
  distanceCenter = (distanceLeft + distanceRight)/2; //For the center of the two wheels

  phi = (distanceRight - distanceLeft)/sizeBetweenWheels;
  positionOfRobot.x = positionOfRobot.x + distanceCenter*cos(currentAngle);
  positionOfRobot.y = positionOfRobot.y + distanceCenter*sin(currentAngle);
  /*
  Serial.print(positionOfRobot.x);
  Serial.print(" ; ");
  Serial.println(positionOfRobot.y);*/

  currentAngle = currentAngle + phi; //New angle for our robot, to calibrate with the compass
}
double sizeHeight = 41; //cm

double distanceToBackcorners = sqrt(sizeBetweenWheels/2 * sizeBetweenWheels/2 + sizeHeight * sizeHeight); //The distance from the start

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
    if(!checkGrass(corners[i]) ||  !checkRocks(corners[i]) || !checkUpperPart(corners[i]) ){
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

int fromProbaToIndex(int first, int second, int third, int randomNumber){/*
  Serial.print("Second plus first : ");
  Serial.print(second+first);
  Serial.print(" ; random number :");
  Serial.println(randomNumber);*/
  if(randomNumber<first)return 0;
  if(first<=randomNumber && randomNumber<(second+first))return 1;
  if((second+first)<=randomNumber && randomNumber<(third+second+first)) return 2;
}
void dodgingObstacle(double distanceToObstacle){
  const int thresholdDistance = 25;
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
    travellingToADestination = true;
    backward = true;
    wasGoingBack = true;
  }
}
void loop() {
  if(count<maxIteration){
    goingHome = count>maxIteration*ratioBeforeGoingHome;
//    Serial.print("Diff on X : ");
   // Serial.print(possibilities[indexPosibility].x - positionOfRobot.x);
  //  Serial.print("Diff on Y : ");
//  Serial.print(" ");
  //  Serial.println(possibilities[indexPosibility].y - positionOfRobot.y);
    odometry();
    if(!travellingToADestination){
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
        travellingToADestination = true;/*
        Serial.print("Proba d'aller a gauche:");
        Serial.print(possibilities[0].howFar/(double)totalFar);
        Serial.print("Proba d'aller au centre:");
        Serial.print(possibilities[1].howFar/(double)totalFar);
        Serial.print("Proba d'aller a droite:");
        Serial.println(possibilities[2].howFar/(double)totalFar);*/
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
          for(int i= 0; i<3;i++){
            distanceI = sqrt(possibilities[i].x * possibilities[i].x + possibilities[i].y*possibilities[i].y);
          /*  Serial.print("Value of distance I : ");
            Serial.println(distanceI);*/
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
    if(travellingToADestination){
      if(goingBack){
        if(time_<40){
           speedWheelRight = -r/4;
           speedWheelLeft = -r/4;
           if(time_==0 || time_==10||time_==20 || time_==30){
            valueX[4*count+time_/10] = positionOfRobot.x;
            valueY[4*count+time_/10] = positionOfRobot.y;
          }
           time_++;

        }
        else{
          travellingToADestination = false;  
          time_ = 0;
          count++; 
          goingBack = false; 

        }
      
    //    positionOfRobot.x += r*cos(currentAngle + PI);
     //   positionOfRobot.y += r*sin(currentAngle + PI);
     }

      else{
        goingBack = false;
        if(time_<40){
          if(indexPosibility==0){
            speedWheelRight = optimalSpeedUpper;
            speedWheelLeft = optimalSpeedLower;
          }
          if(indexPosibility==1){
            speedWheelRight = r/4;
            speedWheelLeft = r/4;            
          }
          if(indexPosibility==2){
            speedWheelRight = optimalSpeedLower;
            speedWheelLeft = optimalSpeedUpper;          
          }
          if(indexPosibility==3 ){
            speedWheelRight = optimalSpeedWheelTurn;
            speedWheelLeft = 0;          
          }
          if(indexPosibility==4){
            speedWheelRight = 0;
            speedWheelLeft = optimalSpeedWheelTurn;          
          }            
         if(time_==0 || time_==10 ||time_==20 || time_==30){
     
      valueX[4*count+time_/10] = positionOfRobot.x;
          valueY[4*count+time_/10] = positionOfRobot.y;
         }
         time_++;

        }
        else{
          time_ =0;
          travellingToADestination = false;
          count++;  

        }
     //  positionOfRobot.x =   possibilities[indexPosibility].x;
      // positionOfRobot.y =   possibilities[indexPosibility].y;
      // currentAngle += angles[indexPosibility];      
      }
     //HERE MUST CHANGE


    }
   // delay(100);  

  }
   if(count==maxIteration ){
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
    count++; 
  }
  
            

}

  
