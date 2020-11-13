
int x = 0;
int y = 0;
const int sizeIndex = 80;
double vitesseRoueLeft = 15; //cm/s
double vitesseRoueRight = 14;//cm/s
double sizeBetweenWheels = 7; //cm
double angle = PI/2; //Initial Angle (delta)
double phi = 0;
double timeBetweenRead = 1;
double distanceLeft = vitesseRoueLeft * timeBetweenRead;
double distanceRight = vitesseRoueRight * timeBetweenRead;
double distanceCenter = (distanceLeft + distanceRight)/2;
int xArray [sizeIndex]; //= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int yArray [sizeIndex]; // = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int index = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {

  if(index<(sizeIndex-1)){
      
  distanceLeft = vitesseRoueLeft * timeBetweenRead;
  distanceRight = vitesseRoueRight * timeBetweenRead;
  distanceCenter = (distanceLeft + distanceRight)/2;

  phi = (distanceRight - distanceLeft)/sizeBetweenWheels;
  x = x + distanceCenter*cos(angle);
  y = y + distanceCenter*sin(angle);
  angle = angle + phi;


    xArray[index] = x;
    yArray[index] = y;
    index++;
  }
  else if(index==(sizeIndex-1)){
        Serial.println("      ");

    Serial.print("x = [");
    for(int i=0;i<(sizeIndex-1);i++){
      Serial.print(xArray[i]);
      Serial.print(",");
      }
      Serial.print(xArray[sizeIndex-1]);
     Serial.println("]");
    Serial.print("y = [");
    for(int i=0;i<(sizeIndex-1);i++){
      Serial.print(yArray[i]);
            Serial.print(",");

      }
            Serial.print(yArray[sizeIndex-1]);

     Serial.println("]");
  index++;
  }
 // delay(1000);

  
}
