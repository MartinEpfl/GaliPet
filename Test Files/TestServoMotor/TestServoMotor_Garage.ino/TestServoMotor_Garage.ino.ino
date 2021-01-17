#include <Servo.h>
//#include <KeyboardController.h>
//USBHost : KeyboardController class

Servo monServomoteur;
int positionOfArm;
int uplim;
int lowlim;
int upspeed;
int downspeed;
int setspeed;
byte incomingByte;


void setup() {
  Serial.begin(9600);
  // Attache le servomoteur à la broche D9

  monServomoteur.attach(9);
  positionOfArm = monServomoteur.read();
  Serial.print(positionOfArm);

  uplim = 70;
  lowlim = 0;
  setspeed = 10;
  upspeed = 0;
  downspeed = 0;

  for (int positionA = positionOfArm; positionA <= uplim; positionA++) {
        monServomoteur.write(positionA);
        Serial.println(monServomoteur.read());
        Serial.println("Init");
        delay(setspeed);
      } 
}

void loop() {

 
 // if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
   
    // say what you got:
    //Serial.print("I received: ");
    //monServomoteur.write(0);
   // Serial.println(incomingByte, DEC);
    Serial.println(monServomoteur.read());
    //if(incomingByte==32){
      Serial.print("Turning...");
      /*
      monServomoteur.write(lowlim);
      delay(1000);
      monServomoteur.write(uplim);
      */
      
      
      for (int position = uplim; position > lowlim; position--) {
        monServomoteur.write(position);
        Serial.println("Clockwise");
        Serial.println(position);

       delay(upspeed);
     // }   
   }      
      delay(1000);

   for (int position = lowlim; position < uplim; position++) {
        monServomoteur.write(position);
               Serial.println("Anti-clockwise");

        delay(downspeed);
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
