
#include <SharpIR.h>

#include <SharpIR.h>

#include <SharpIR.h>


/*  Project: Ardu_Serie # 82
 *   Sharp IR Sensor -  0A41SK 
 *   4–30cm Infrared IR Distance Measuring Sensor
 *   
 *   INO file: _82_Sharp_Distance_Sensor_01.ino
 *   
 *   date: 8/27/19
 *  
 *  code by: https://github.com/guillaume-rico
 *  hardware by: GP2Y0A02YK0F IRSharp (https://www.pololu.com/product/2464)
       software: Arduino IDE 1.8.9
 * 
 *  Description: There is no easy way more simple than that sketch to test right away GP2Y0A02YK0F IRSharp!
 *               Please open Serial Monitor at 9600 bps:)
 *  
 *  Visit: https://medium.com/jungletronics
 *         SHARP GP2Y0A710K0F IR sensor with Arduino and SharpIR library example code
 *         More info: https://www.makerguides.com 
 *  
 *  Tutorial: https://medium.com/jungletronics/sharp-ir-sensor-0a41sk-41c2b3848e39
 *  
 *  License: CC-SA 3.0, feel free to use this code however you'd like.
 *  Please improve upon it! Let me know how you've made it better.
 */
/**/
                        // Include the library:
#include <SharpIR.h>
                        // Define model and input pin:
#define NWIRPin A1
#define NEIRPin A2
#define SWIRPin A3
#define SEIRPin A4

#define model 1080

                        // Create variable to store the distance:
int distanceNW;
int distanceNE;
int distanceSW;
int distanceSE;

/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/
                        // Create a new instance of the SharpIR class:
SharpIR sensorNW = SharpIR(NWIRPin, model);
SharpIR sensorNE = SharpIR(NEIRPin, model);
SharpIR sensorSW = SharpIR(SWIRPin, model);
SharpIR sensorSE = SharpIR(SEIRPin, model);


int const numberOfInputs = 20;
int count = 0;
int average = 0;

int numberDetected[numberOfInputs];
void setup() {
                        // Begin serial communication at a baud rate of 9600:
  Serial.begin(9600);

  

}
void loop() {
                        // Get a distance measurement and store it as distance_cm:
  distanceNW = sensorNW.distance();
  distanceNE = sensorNE.distance();
  distanceSW = sensorSW.distance();
  distanceSE = sensorSE.distance();
  //Serial.print(distanceNW);
  Serial.print(" ");
  //Serial.print(distanceNE);
  Serial.print(" ");
 // Serial.print(distanceSW);
  Serial.print(" ");
  if(distanceNW<=80 and distanceNW>=5 and count!=numberOfInputs){
    numberDetected[count] = distanceNW;
    /*Serial.println("This is numberDetected :");
    Serial.println(numberDetected[count] );*/
    count++;
  }
  if(count == numberOfInputs){
      average =0;
      for(int j=0;j<numberOfInputs;j++){
        average+=numberDetected[j];
      }
      average/=numberOfInputs;
      count = 0;
  }
  Serial.println(average);
 // Serial.println(distanceSE);
                        // Print the measured distance to the serial monitor:
  if(distanceSE<80 and distanceSE>10 and distanceSW<80 and distanceSW>10){
    if(distanceNE<80 and distanceNE>10 and distanceNW<80 and distanceNW>10){
   //  Serial.println("Parpaing found!");
    }
     else{ //Serial.println("Bottle found!");
     }
    }
  delay(100);
  }
/*view raw_82_Sharp_Distance_Sensor_01.ino hosted with ❤ by GitHub
*/
