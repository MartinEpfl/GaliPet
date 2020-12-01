typedef struct {
    double x;
    double y;
    boolean canGoThere = true;
    int howFar = 0;
} position_;    

const int sizeBadArea = 300; //3 meters for each arena we don't want to go in
const int sizeOfFullArena = 800;
const double epsilon = 20; //How close you dont want to get close to the area you don't want to go in
const double r = 40; //Radius of circle

position_ possibilities[3];
const double angles[3] = {PI/4, 0,-PI/4,};
position_ positionOfRobot;
double currentAngle = PI/4;
bool destinationAvailable=false;
boolean travellingToADestination = false;
int indexPosibility;
int count= 0;
const int maxIteration = 100;
bool goingBack = false;
bool wasGoingBack = false;
bool goingHome = false;
double ratioBeforeGoingHome = 0.85 ;
int totalFar = 0;
double valueX[4*maxIteration];
double valueY[4*maxIteration];
int time_ = 0;
//Odometry 69*PI/16; 31*PI/16;
double speedWheelRight = 0; //cm/s
double speedWheelLeft = 0;
double sizeBetweenWheels = 38; //cm
double timeBetweenRead = 1;
double distanceLeft = speedWheelLeft * timeBetweenRead/100;
double distanceRight = speedWheelRight * timeBetweenRead/100;
double distanceCenter = (distanceLeft + distanceRight)/2;
double phi = 0;

const int optimalSpeedLower = (r-sizeBetweenWheels/2)*(PI/4)/4;
const int optimalSpeedUpper = (r+sizeBetweenWheels/2)*(PI/4)/4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(41);

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

bool checkIfCanGo(position_ destination){
  if(destination.x<(sizeBadArea+epsilon)  && destination.y>(sizeOfFullArena-sizeBadArea-epsilon)){ //Rock area
    return false;
  }
  if(destination.x>(sizeOfFullArena-sizeBadArea-epsilon) && destination.y>(sizeOfFullArena-sizeBadArea-epsilon)){ //Upper ramp area
    return false;
  }
  if(destination.x>(sizeOfFullArena-sizeBadArea-epsilon) && destination.y<(sizeBadArea+epsilon)){ //Grass area
    return false;
  }
  if(destination.x<epsilon || destination.x>sizeOfFullArena-epsilon || destination.y<epsilon || destination.y>sizeOfFullArena-epsilon){ //Don't get out of the arena
    return false;
  }
  return true; //Otherwise it is OK
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
        totalFar+=possibilities[i].howFar;/*
        Serial.print("Nombre de possibilité pour la position ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(possibilities[i].howFar);*/


        if(possibilities[i].canGoThere && (!wasGoingBack || i!=1)){
          destinationAvailable=true;
          wasGoingBack = false;
        }
      }
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
         //   int random_ = random(0,totalFar);
           // indexPosibility = fromProbaToIndex(possibilities[0].howFar,possibilities[1].howFar,possibilities[2].howFar,random_);
            indexPosibility = random(0,3);
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
           speedWheelRight = -50/4;
           speedWheelLeft = -50/4;
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

  
