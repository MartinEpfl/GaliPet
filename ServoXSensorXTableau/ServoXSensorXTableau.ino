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
int numberOfInputs = 20;
int numberDetected[numberOfInputs];
int average = 0;
void setup() {
  Serial.begin(9600);
  // Attache le servomoteur à la broche D9

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
      for(j=0;j<numberOfInputs;++){
        average+=numberDetected[j];
      }
      average/=numberOfInputs;
      fromTo();
  }
  Serial.print("DEBUGGING!!Distance : ");
  Serial.print(distance_cm);
  Serial.print("count is : ");
  Serial.println(count);


  /*
  
  for (int position = 0; position <= 180; position++) {
    monServomoteur.write(position);
    delay(15);
  }
  //Fait bouger le bras de 0° à 180°

  // Fait bouger le bras de 180° à 10°
  for (int position = 180; position >= 0; position--) {
    monServomoteur.write(position);
    delay(15);
  }*/
}

void fromTo(){
  positionOfArm = monServomoteur.read();
  average = (average-3)/17*180;
  print("Debugging fromTo(), this is average : ");
  print(average)
  print(" and this is positionOfArm: ");
  println(positionOfArm)
  if(positionOfArm>average){
      for (i = average; i<= positionOfArm; i++) {
        monServomoteur.write(i);
        delay(8);
      }      
  }
  else{
      for (i = average; i>= positionOfArm; i--) {
        monServomoteur.write(i);
        delay(8);
      }   
  }
}
