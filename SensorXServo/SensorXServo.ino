#include <SharpIR.h>

/* Initialize sensor
*/
                        // Include the library:

                        // Define model and input pin:
#define IRPin A5
#define model 430

int distance_cm; // Create variable to store the distance

SharpIR mySensor = SharpIR(IRPin, model); // Create a new instance of the SharpIR class:

/* Initialize servo
*/

#include <Servo.h>
Servo monServomoteur;
int positionOfArm;
byte incomingByte;

/* Put your setup code here, to run once:
*/
void setup() {
  

  // Begin serial communication at a baud rate of 9600:
  Serial.begin(9600);

  // Initialize servomotor
  monServomoteur.attach(7);
  positionOfArm = monServomoteur.read();
  Serial.print(positionOfArm);

  for (int position = 0; position <= 90; position++) {
        monServomoteur.write(position);
        delay(8);
      } 

}

/* Put your main code here, to run repeatedly:
*/

void loop() {
  // 

  distance_cm = mySensor.distance();
  // Print the measured distance to the serial monitor:
  Serial.print("Mean distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  if(distance_cm <= 20){
      Serial.println("Turning...");

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
