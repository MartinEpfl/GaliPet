//DATASHEET https://www.elechouse.com/elechouse/images/product/GY-26-USART%20Digital%20Compass/gy-26%20manual.pdf
char valeurByte[8];
int angle = 0;
int stack =0;
boolean readByte = false;
int waitingBetweenAngle = 300; //Waiting time between output each value
boolean value = true;
const int numberOfValues = 5;
char oldValues[numberOfValues ];
int index = 0;
const int strictOnCompassSetUp = 2;
bool needToSetUp = true;
  bool canExit = true;

double tempAngle;
void setup() {

  Serial.begin(9600);
  Serial2.begin(9600); //Compass has a Baud Rate of 9600
  gettingCompassReady();
}

void loop() {

  Serial.print("Value of the angle : ");
  Serial.println(updateCompassValue());
}

double updateCompassValue(){
  double average = 0;
  for(int i=0;i<numberOfValues;i++){
    average+= oldValues[i];
  }
  average/=numberOfValues; 
  do{ 
    canExit = true;
    tempAngle = getCompassValue();
    if(abs(tempAngle-average)>strictOnCompassSetUp){
      canExit = false;
      delay(waitingBetweenAngle);
  }
  }while(!canExit);
  oldValues[index] = tempAngle;
  index++;
  index = index % numberOfValues;
  return tempAngle;
}

double getCompassValue() {

  value = false;
  Serial2.write(0x31); //Asking for the angle, for each command sent you get 8 byte as an answer
  //First byte, enter => New Line => hundreds of angle => tens of angle => bits of angle => Decimal point of angle => Decimal of angle => Calibrate sum
  while (!value) {
    Serial.println("STUCK");
    Serial2.write(0x31); ///NEEDED?
    if (Serial2.available()) {
      valeurByte[stack] = Serial2.read(); //Read the value & stacks it
      stack = (stack + 1) % 8; //Allows to read the full 8 bytes
      if (stack == 0) {
        angle = (valeurByte[2] - 48) * 100 + (valeurByte[3] - 48) * 10 + (valeurByte[4] - 48); //Computes the angle using the read bytes 
        if(angle>=0 and angle<360){
          value = true;
        }
        else{
          delay(waitingBetweenAngle);
        }  
      }
    }
  }
  return angle;

}

void gettingCompassReady(){
  bool canExit = true;
  int index = 0;
  double value = 0;
  double average = 0;
  for(int i=0;i<numberOfValues;i++){
    oldValues[i] = getCompassValue();
    delay(waitingBetweenAngle);
  }
  do{
    delay(waitingBetweenAngle);
    value = getCompassValue();
    canExit = true;
    oldValues[index] = value;
    index++;
    index = index % numberOfValues;
    for(int i=0;i<numberOfValues;i++){
      average+= oldValues[i];
    }
    average/=numberOfValues;
    for(int i=0;i<numberOfValues;i++){
      if(abs(oldValues[i]-average)>strictOnCompassSetUp){
        canExit = false;
      }
    }     
  }while(!canExit);
  Serial.println("COMPASS IS READY!!");
}
