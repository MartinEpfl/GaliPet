
#include <Servo.h>
//#include <KeyboardController.h>
//USBHost : KeyboardController class

Servo monServomoteur;
int positionOfArm;
byte incomingByte;


void setup() {
  Serial.begin(9600);
  // Attache le servomoteur à la broche D9

  monServomoteur.attach(7);
  positionOfArm = monServomoteur.read();
  Serial.print(positionOfArm);
}

void loop() {
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
   
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
    if(incomingByte==32){
      Serial.print("Turning...");

      for (int position = 0; position <= 180; position++) {
        monServomoteur.write(position);
        delay(8);
      }   
      delay(1000);
      for (int position = 180; position >= 0; position--) {
        monServomoteur.write(position);
        delay(8);
      }    
    }     
  }
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
