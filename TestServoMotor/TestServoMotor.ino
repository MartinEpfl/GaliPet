
#include <Servo.h>
#include <Keyboard.h>

Servo monServomoteur;

void setup() {
  Serial.begin(9600);
  Keyboard.begin();
  // Attache le servomoteur à la broche D9

  monServomoteur.attach(9);
}

void loop() {
  
  int in= 0; // for incoming serial data

  if(Serial.available()){
    in = Serial.read();
   
    Serial.print(F("Input detected: '"));
    Serial.print(in);
    Serial.print(F("' 0x"));
    Serial.print(in, HEX);
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
