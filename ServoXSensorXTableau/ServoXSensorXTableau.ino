#include <SharpIR.h>

#include <Servo.h>
//#include <KeyboardController.h>
//USBHost : KeyboardController class
#define IRPin A5
#define model 430

                        // Create variable to store the distance:
int distance_cm;
Servo monServomoteur;
int positionOfArm;
int count = 0;
SharpIR mySensor = SharpIR(IRPin, model);
int const numberOfInputs = 20;
int numberDetected[numberOfInputs];
int average = 0;
void setup() {
  Serial.begin(9600);
  monServomoteur.attach(7);
  positionOfArm = monServomoteur.read();
  Serial.print(positionOfArm);
}

void loop() {

  distance_cm = mySensor.distance();
  if(distance_cm<=20 and distance_cm>=3 and count!=numberOfInputs){
    numberDetected[count] = distance_cm;
    count++;
  }
  if(count == numberOfInputs){
      average =0;
      for(int j=0;j<numberOfInputs;j++){
        average+=numberDetected[j];
      }
      average/=numberOfInputs;
      fromTo();
      count = 0;
  }
  Serial.print("DEBUGGING!!Distance : ");
  Serial.print(distance_cm);
  Serial.print("count is : ");
  Serial.println(count);


}

void fromTo(){
  positionOfArm = monServomoteur.read();
  average = (average-3)/17*180; //Brings average between 0 and 180
  Serial.print("Debugging fromTo(), this is average : ");
  Serial.print(average);
  Serial.print(" and this is positionOfArm: ");
  Serial.println(positionOfArm);
  if(positionOfArm>average){
      for (int i = positionOfArm; i>= average; i--) {
        monServomoteur.write(i);
        delay(8);
      }      
  }
  else{
      for (int i = positionOfArm; i<= average; i++) {
        monServomoteur.write(i);
        delay(8);
      }   
  }
}
