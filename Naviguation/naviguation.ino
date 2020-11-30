typedef struct {
    double x;
    double y;
    boolean canGoThere = true;
} position_;    

const int sizeBadArea = 300; //3 meters for each arena we don't want to go in
const int sizeOfFullArena = 800;
const double epsilon = 10; //How close you dont want to get close to the area you don't want to go in
const double r = 20; //Radius of circle
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
double ratioBeforeGoingHome = 0.8 ;

double valueX[maxIteration];
double valueY[maxIteration];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(45);
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

void loop() {

  if(count<maxIteration){
    goingHome = count>maxIteration*ratioBeforeGoingHome;
    if(!travellingToADestination){
      destinationAvailable=false;
      for(int i=0;i<3;i++){
        possibilities[i].x = positionOfRobot.x + r*cos(angles[i]+currentAngle);
        possibilities[i].y = positionOfRobot.y + r*sin(angles[i]+currentAngle); /*
        Serial.print(possibilities[i].x);
        Serial.print( " , ");
        Serial.println(possibilities[i].y);*/
        possibilities[i].canGoThere = checkIfCanGo(possibilities[i]);
        if(possibilities[i].canGoThere && (!wasGoingBack || i!=1)){
          destinationAvailable=true;
        }
      }
      if(destinationAvailable){
                  travellingToADestination = true;

        if(!goingHome){
          do{
            indexPosibility = random(0,3);
  
          }while(!possibilities[indexPosibility].canGoThere);
          wasGoingBack = false;
        }
        else{
          int indexToHome = 0;
          int distanceMin = 1131; //sqrt(2*800*800)
          int distanceI;
          for(int i= 0; i<3;i++){
            distanceI = sqrt(possibilities[i].x * possibilities[i].x + possibilities[i].y*possibilities[i].y);
            if(distanceI<distanceMin){
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
        positionOfRobot.x += 10*cos(currentAngle + PI);
        positionOfRobot.y += 10*sin(currentAngle + PI);
      }
      else{
       positionOfRobot.x =   possibilities[indexPosibility].x;
       positionOfRobot.y =   possibilities[indexPosibility].y;
       currentAngle += angles[indexPosibility];      
      }
     //HERE MUST CHANGE

     valueX[count] = positionOfRobot.x;
     valueY[count] = positionOfRobot.y;
     travellingToADestination = false;
     goingBack = false;
     count++;  
    }
   // delay(100);  

  }
   if(count==maxIteration ){
    Serial.println("DONE!");
    Serial.print("x = [");
    for(int i=0;i<(maxIteration-1);i++){
      Serial.print(valueX[i]);
      Serial.print(",");
    }
    Serial.print(valueX[maxIteration-1]);
    Serial.println("]");
    Serial.print("y = [");
    for(int i=0;i<(maxIteration-1);i++){
      Serial.print(valueY[i]);
      Serial.print(",");
    }
    Serial.print(valueY[maxIteration-1]);
    Serial.println("]");
    count++; 
  }
  
            

}

  
