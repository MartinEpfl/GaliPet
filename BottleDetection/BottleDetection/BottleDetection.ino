
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
#define L_IRPin A1
#define M_IRPin A2
#define R_IRPin A3
#define T_IRPin A4
#define B_IRPin A5

#define model 1080

                        // Create variable to store the distance:
int distanceL;
int distanceM;
int distanceR;
int distanceT;
int distanceB;

                        //Tuning parameters

int cutOffDistance = 200;

/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/
                        // Create a new instance of the SharpIR class:
SharpIR sensorL = SharpIR(L_IRPin, model);
SharpIR sensorM = SharpIR(M_IRPin, model);
SharpIR sensorR = SharpIR(R_IRPin, model);
SharpIR sensorT = SharpIR(T_IRPin, model);
SharpIR sensorB = SharpIR(B_IRPin, model);


int const numberOfInputs = 20;
int count = 0;
int average = 0;

int numberDetected[numberOfInputs];


#define WINDOW_SIZE 50
unsigned long INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;



void setup() {
                        // Begin serial communication at a baud rate of 9600:
  Serial.begin(9600);

}
void loop() {
                        // Get a distance measurement and store it as distance_cm:
  distanceL = sensorL.distance();
  distanceM = sensorM.distance();
  distanceR = sensorR.distance();
  distanceT = sensorT.distance();
  distanceB = sensorT.distance();
SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = distanceM;        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

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


if (sensorT.distance()>cutOffDistance) { //No obstacle detected --> T=0
  if (sensorM.distance()>cutOffDistance) { //Nothing on middle sensor --> T=0 M=0
    if (sensorL.distance()>cutOffDistance) { // Noting on left sensor --> T=0 M=0 L=0
      if (sensorR.distance()<cutOffDistance) { // Right sensor detects --> T=0 M=0 L=0 R=1
        Serial.println("Turn left until M=1");
      }
      else {
        Serial.println("Nothing detected"); // Nothing detected --> T=0 M=0 L=0 R=0
      }
    }
    else { // Left sensor detects --> T=0 M=0 L=1
      if (sensorR.distance()>cutOffDistance) { // Nothing on right sensor --> T=0 M=0 L=1 R=0
        Serial.println("Turn right until M=1");
      }
    }
  }
  else { //Middle sensor detects --> T=0 M=1
    if (sensorL.distance()<cutOffDistance) { // Left sensor detects --> T=0 M=1 L=1
      if (sensorR.distance()<cutOffDistance) { // Right sensor detects --> T=0 M=1 L=1 R=1
        if (sensorB.distance()<cutOffDistance) { // Bottom sensor detects --> T=0 M=1 L=1 R=1 B=1
          Serial.println("Catch bottle");
        }
        else { // Nothing on bottom detector --> T=0 M=1 L=1 R=1 B=1
          Serial.println("Forward slowly");
        }
      }
      else { // Nothing on right detector --> T=0 M=1 L=1 R=0
        Serial.println("Turn right and forward");
      }
    }
    else { // Nothing on left sensor --> T=0 M=1 L=0
      if (sensorR.distance()<cutOffDistance) { // Right sensor detects --> T=0 M=1 L=0 R=1
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
