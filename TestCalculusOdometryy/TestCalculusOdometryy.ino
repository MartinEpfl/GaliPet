//https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf

//Position of the center of the two wheels
double x = 0;
double y = 0;
const int sizeIndex = 80;
double vitesseRoueLeft = 15; //cm/s
double vitesseRoueRight = 10;//cm/s
double sizeBetweenWheels = 7; //cm
double angle = PI/2; //Initial Angle (delta)
double phi = 0;
double timeBetweenRead = 1;
double distanceLeft = vitesseRoueLeft * timeBetweenRead;
double distanceRight = vitesseRoueRight * timeBetweenRead;
double distanceCenter = (distanceLeft + distanceRight)/2;
double xArray [sizeIndex]; 
double yArray [sizeIndex]; 
int index = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {

  if(index<(sizeIndex-1)){
      
  distanceLeft = vitesseRoueLeft * timeBetweenRead; //Distance from the left wheel since last check
  distanceRight = vitesseRoueRight * timeBetweenRead; //for the right wheel
  distanceCenter = (distanceLeft + distanceRight)/2; //For the center of the two wheels

  phi = (distanceRight - distanceLeft)/sizeBetweenWheels;
  x = x + distanceCenter*cos(angle);
  y = y + distanceCenter*sin(angle);
  angle = angle + phi; //New angle for our robot, to calibrate with the compass


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
  
}
