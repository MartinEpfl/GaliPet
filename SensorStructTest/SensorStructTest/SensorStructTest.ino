#include <SharpIR.h>


#define model80 1080
#define model30 430
#define pin A13



const int window_size = 5;

struct sensor{ 
  double read_value = 0;
  double values_sensor[window_size];
  int index = 0;
  double average = 0;
  int pinSensor;
  SharpIR* mySensor;
  
  void sensorInit(int whichPin, int maxDistance){
      pinSensor = whichPin;
      if(maxDistance ==30){
        mySensor = new SharpIR(pinSensor, model30);
      }
      else if(maxDistance == 80){
         mySensor = new SharpIR(pinSensor, model80);
      }
      else{
         mySensor = new SharpIR(pinSensor, model30);
      }
        
  }
  void loop_sensor(){
    read_value =  mySensor->distance(); //analogRead(pinSensor) - initSmooth;
    values_sensor[index] = read_value;
    index++;
    index = index%window_size;
  }

  double get_value(){
    average = 0;
    for(int i=0; i<window_size;i++){
      average += values_sensor[i];
    }
    return average/window_size;  
  }
};

const int numberOfSensorsBack = 2;
sensor sensorsBack[numberOfSensorsBack];

int pinsBack[] = {12,13};

const int numberOfSensorObstacle = 1;


sensor sensorsObstacle[numberOfSensorObstacle];



const int numberOfSensorsFront = 4;

sensor sensorsFront[numberOfSensorsFront];
int pinsFront[] = {A1, A2, A3, A4}; //The sensors from 0 to 3 are left, middle, right, top according to the robots pov
int cutOffDistance = 200; //Value used to check if there is something in front of the sensor (in a binary way)
int veryCloseDistance = 20; //Value used to check if something is very close to the IR sensors



void setup() {
  Serial.begin(9600);
  for(int i=0; i<numberOfSensorsBack;i++){
    sensorsBack[i].sensorInit(pinsBack[i], 30);
  }

  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].sensorInit(pinsFront[i], 80);
  }

}

void loop() {
  for(int i=0;i<1;i++){
    sensorsBack[i].loop_sensor();

    delay(10);
  }
  for (int i = 0; i < numberOfSensorsFront; i++) {
    sensorsFront[i].loop_sensor();
  }

    Serial.print(sensorsFront[0].get_value());
  Serial.print("  ");
  Serial.print(sensorsFront[1].get_value());
  Serial.print("  ");
  Serial.print(sensorsFront[2].get_value());
  Serial.print("  ");
  Serial.println(sensorsFront[3].get_value());

    
}
