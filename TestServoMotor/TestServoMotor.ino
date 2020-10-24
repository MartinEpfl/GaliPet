
#include <Servo.h>
#include <Keyboard.h>
#include <KeyboardController.h>
//USBHost : KeyboardController class

Servo monServomoteur;
USBHost usb;
KeyboardController keyboard(usb);
int position;
void setup() {
  Serial.begin(9600);
  // Attache le servomoteur à la broche D9

  monServomoteur.attach(9);
  position = monServomoteur.read();

}

void loop() {
  
  usb.Task();

  
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

void keyPressed() {
  Serial.print("Pressed:  ");
  Serial.print(keyboard.getKey());
  if(keyboard.getKey() == 'd' and position<180)
    position++;
  if(keyboard.getKey() == 'f' and position>0)
    position--;  
  monServomoteur.write(position);

}
