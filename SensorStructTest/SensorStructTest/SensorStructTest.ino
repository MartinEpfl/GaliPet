#include <SharpIR.h>


#define model80 1080
#define model30 430
#define pin A13



const int window_size = 20;

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



void setup() {
  Serial.begin(9600);
  for(int i=0; i<numberOfSensorsBack;i++){
    sensorsBack[i].sensorInit(pinsBack[i], 30);
  }
}

void loop() {
  for(int i=0;i<1;i++){
    sensorsBack[i].loop_sensor();
    Serial.print("Value of sensor ");
    Serial.print(i);
    Serial.print(" is ");
    Serial.println(sensorsBack[i].get_value());
    delay(10);
  }
}
