//DATASHEET https://www.elechouse.com/elechouse/images/product/GY-26-USART%20Digital%20Compass/gy-26%20manual.pdf
char valeurByte[8];
int angle = 0;
int stack =0;
boolean readByte = false;
int waitingBetweenAngle = 300; //Waiting time between output each value
boolean value = true;
const int numberOfValues = 15;
double oldValues[numberOfValues ];
int index = 0;
const int strictOnCompassSetUp = 2;
bool needToSetUp = true;
  bool canExit = true;

double tempAngle;
void setup() {

  Serial.begin(9600);
  Serial3.begin(9600); //Compass has a Baud Rate of 9600
  gettingCompassReady();
}

void loop() {

  Serial.print("Value of the angle : ");
  Serial.println(updateCompassValue());
  delay(waitingBetweenAngle);
}

double updateCompassValue(){
  Serial.println("Updating the array :");
  double average = 0;
  for(int i=0;i<numberOfValues;i++){
    average+= oldValues[i];
  }
  average/=numberOfValues; 
  do{ 
    canExit = true;
    tempAngle = getCompassValue();
    if(abs(tempAngle-average)>strictOnCompassSetUp){
        Serial.print("Average is :");
        Serial.print(average);
        Serial.print("and the value of the angle read is : ");
        Serial.println(tempAngle);
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
  Serial3.write(0x31); //Asking for the angle, for each command sent you get 8 byte as an answer
  //First byte, enter => New Line => hundreds of angle => tens of angle => bits of angle => Decimal point of angle => Decimal of angle => Calibrate sum
  while (!value) {
   // Serial.println("STUCK");
   // Serial3.write(0x31); ///NEEDED?
    if (Serial3.available()) {
      //Serial.println("INSIDE");
      valeurByte[stack] = Serial3.read(); //Read the value & stacks it
      stack = (stack + 1) % 8; //Allows to read the full 8 bytes
      if (stack == 0) {
        angle = (valeurByte[2] - 48) * 100 + (valeurByte[3] - 48) * 10 + (valeurByte[4] - 48); //Computes the angle using the read bytes 
        if(angle>=0 and angle<360){
          value = true;
          Serial.print("VALUE AT THE ROOT IS : ");
          Serial.println(angle);
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
  double value = 0;
  double average = 0;
  Serial.println("Getting compass ready");
  for(int i=0;i<numberOfValues;i++){

    oldValues[i] = getCompassValue();
        Serial.print(" coéputing the average : ");
    Serial.println(oldValues[i]);
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
        Serial.print("Average is :");
        Serial.print(average);
        Serial.print("and the value of the array is : ");
        Serial.print(oldValues[i]);
        Serial.print(" for a value of i : ");
        Serial.println(i);
        canExit = false;
      }
    }     
  }while(!canExit);
  Serial.println("COMPASS IS READY!!");
}
