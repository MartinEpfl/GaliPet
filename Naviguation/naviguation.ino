typedef struct {
    double x;
    double y;
    boolean canGoThere = true;
} position_;    

const int sizeBadArea = 300; //3 meters for each arena we don't want to go in
const int sizeOfFullArena = 800;
const double epsilon = 10; //How close you dont want to get close to the area you don't want to go in
const double r = 50; //Radius of circle
position_ possibilities[5];
const double angles[5] = {3*PI/4, PI/4, 0,-PI/4,-3*PI/4};
position_ positionOfRobot;
double currentAngle = PI/4;
bool destinationAvailable=false;
boolean travellingToADestination = false;
int indexPosibility;
int count= 0;
double valueX[100];
double valueY[100];
int indexStuck = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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

  if(count<100){
    if(!travellingToADestination){
      destinationAvailable=false;
      for(int i=0;i<5;i++){
        possibilities[i].x = positionOfRobot.x + r*cos(angles[i]+currentAngle);
        possibilities[i].y = positionOfRobot.y + r*sin(angles[i]+currentAngle); /*
        Serial.print(possibilities[i].x);
        Serial.print( " , ");
        Serial.println(possibilities[i].y);*/
        possibilities[i].canGoThere = checkIfCanGo(possibilities[i]);
        if(possibilities[i].canGoThere){
          destinationAvailable=true;
        }
      }
      if(destinationAvailable){
        indexStuck = 0;
        do{
          indexPosibility = rand()%5;
          indexStuck++;
        }while(!possibilities[indexPosibility].canGoThere);
                 

        travellingToADestination = true;
      }
    }
    if(travellingToADestination){
     //HERE MUST CHANGE
     positionOfRobot.x =   possibilities[indexPosibility].x;
     positionOfRobot.y =   possibilities[indexPosibility].y;
     currentAngle += angles[indexPosibility];
     valueX[count] = positionOfRobot.x;
     valueY[count] = positionOfRobot.y;
     travellingToADestination = false;
     count++;  
    }
   // delay(100);  

  }
   if(count==100 ){
    Serial.println("DONE!");
    Serial.print("x = [");
    for(int i=0;i<(99);i++){
      Serial.print(valueX[i]);
      Serial.print(",");
    }
    Serial.print(valueX[99]);
    Serial.println("]");
    Serial.print("y = [");
    for(int i=0;i<(99);i++){
      Serial.print(valueY[i]);
      Serial.print(",");
    }
    Serial.print(valueY[99]);
    Serial.println("]");
    count++; 
  }
  
            

}

  
