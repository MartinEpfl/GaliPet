//DATASHEET https://www.elechouse.com/elechouse/images/product/GY-26-USART%20Digital%20Compass/gy-26%20manual.pdf
char valeurByte[8];
int angle = 0;
int stack =0;
boolean readByte = false;
int waitingBetweenAngle = 300; //Waiting time between output each value
boolean value = true;
void setup() {

  Serial.begin(9600);
  Serial2.begin(9600); //Compass has a Baud Rate of 9600

}

void loop() {

  compass();

}

void compass() {

  value = false;
  Serial2.write(0x31); //Asking for the angle, for each command sent you get 8 byte as an answer
  //First byte, enter => New Line => hundreds of angle => tens of angle => bits of angle => Decimal point of angle => Decimal of angle => Calibrate sum
  while (!value) {
    Serial.println("STUCK");
    Serial2.write(0x31);
    if (Serial2.available()) {
      valeurByte[stack] = Serial2.read(); //Read the value & stacks it
      stack = (stack + 1) % 8; //Allows to read the full 8 bytes
      if (stack == 0) {
        angle = (valeurByte[2] - 48) * 100 + (valeurByte[3] - 48) * 10 + (valeurByte[4] - 48); //Computes the angle using the read bytes 
        value = true;
      }
    }
  }
  Serial.println(angle);
  delay(waitingBetweenAngle);
}
