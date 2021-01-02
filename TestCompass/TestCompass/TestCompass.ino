/*
  Gy-26 - Compass.
  Criado por Igor Araujo - www.igoraraujo.eng.br - 2012
*/

#define SWIRPin A4
#define model 1080

char valorbyte[8];
int graus = 0;
int contador = 0;
byte valor = 0;
double diff = 0;
double angle = 0;
double distanceNE;

void setup() {
  //  servoArm.attach(pinservoArm);
  //servoArm.write(20);
  //servoBack.attach(pinServoBack);

  Serial.begin(19200);
  Serial2.begin(9600);
  
  readValueCompass();

  diff = PI / 4 - angle;
}

void loop() {

  readValueCompass();
  //Serial.println(angle);
  delay(10);
 
}

void readValueCompass() {
  char valeurByte[8];
  int stack = 0;
  boolean readByte = false;
  boolean value = false;
  double tempAngleCompass;
  Serial2.write(0x31); //Asking for the angle, for each command sent you get 8 byte as an answer
  //First byte, enter => New Line => hundreds of angle => tens of angle => bits of angle => Decimal point of angle => Decimal of angle => Calibrate sum
  while (!value) {
    //  Serial.println("stuck");
    if (Serial2.available()) {
      valeurByte[stack] = Serial2.read(); //Read the value & stacks it
      stack = (stack + 1) % 8; //Allows to read the full 8 bytes
      if (stack == 0) {
        tempAngleCompass = (valeurByte[2] - 48) * 100 + (valeurByte[3] - 48) * 10 + (valeurByte[4] - 48); //Computes the angle by reading bytes
        value = true;
      }
    }
  }

  //        Serial.print("THIS IS THE ANGLE FROM THE COMPASS VALUE : ");
  Serial.println(tempAngleCompass);
}
