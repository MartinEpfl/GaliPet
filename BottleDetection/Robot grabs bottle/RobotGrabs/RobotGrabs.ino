// Include the library:
#include <SharpIR.h>


const int window_size = 20;

struct sensor {
  double read_value = 0;
  double values_sensor[window_size];
  int index = 0;
  double average = 0;
  int pinSensor;
  SharpIR* mySensor;

  void sensorInit(int whichPin, int maxDistance) {
    pinSensor = whichPin;
    if (maxDistance == 30) {
      mySensor = new SharpIR(pinSensor, model30);
    }
    else if (maxDistance == 80) {
      mySensor = new SharpIR(pinSensor, model80);
    }
    else {
      mySensor = new SharpIR(pinSensor, model30);
    }

  }
  void loop_sensor() {
    read_value =  mySensor->distance(); //analogRead(pinSensor) - initSmooth;
    values_sensor[index] = read_value;
    index++;
    index = index % window_size;
  }

  double get_value() {
    average = 0;
    for (int i = 0; i < window_size; i++) {
      average += values_sensor[i];
    }
    return average / window_size;
  }
};



// Define model and input pin:

#define model80 1080
#define model30 430
const int numberOfSensorsFront = 2;
sensor sensorsFront[numberOfSensorsFront];

int pinsFront[] = {A1, A2, A3, A4};


//Tuning parameters

int cutOffDistance = 200;


void setup() {
  // Begin serial communication at a baud rate of 9600:
  Serial.begin(9600);
  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].sensorInit(pinsFront[i], 30);
  }
}
void loop() {

  /*
    Serial.print(distanceL);
    Serial.print(" ");
    Serial.print(distanceM);
    Serial.print(" ");
    Serial.print(distanceR);
    Serial.print(" ");
    Serial.print(distanceT);
    Serial.print(" ");
    Serial.println(distanceB);
  */
  //Serial.print(" ");
  //Serial.println(distanceT);


  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].loop_sensor();
  }


  if (sensorsFront[3].get_value() > cutOffDistance) { //No obstacle detected --> T=0
    if (sensorsFront[1].get_value() > cutOffDistance) { //Nothing on middle sensor --> T=0 M=0
      if (sensorsFront[0].get_value() > cutOffDistance) { // Noting on left sensor --> T=0 M=0 L=0
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=0 L=0 R=1
          Serial.println("Turn left until M=1");
        }
        else {
          Serial.println("Nothing detected"); // Nothing detected --> T=0 M=0 L=0 R=0
        }
      }
      else { // Left sensor detects --> T=0 M=0 L=1
        if (sensorsFront[2].get_value() > cutOffDistance) { // Nothing on right sensor --> T=0 M=0 L=1 R=0
          Serial.println("Turn right until M=1");
        }
      }
    }
    else { //Middle sensor detects --> T=0 M=1
      if (sensorsFront[0].get_value() < cutOffDistance) { // Left sensor detects --> T=0 M=1 L=1
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=1 L=1 R=1
          Serial.println("Catch bottle");
        }

        else { // Nothing on right detector --> T=0 M=1 L=1 R=0
          Serial.println("Turn right and forward");
        }
      }
      else { // Nothing on left sensor --> T=0 M=1 L=0
        if (sensorsFront[2].get_value() < cutOffDistance) { // Right sensor detects --> T=0 M=1 L=0 R=1
          Serial.println("Turn left and forward");
        }
        else { // Nothing on Right sensor --> T=0 M=1 L=0 R=0
          Serial.println("Go forward");
        }
      }
    }
  }
  else { // Top sensor detects --> T=1
    Serial.println("Obstacle detected");
  }


  delay(100);

}
