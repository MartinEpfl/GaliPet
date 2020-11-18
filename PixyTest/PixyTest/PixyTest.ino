// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP SPI port.  For more information go here:
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//
  
#include <Pixy2.h>
float focalLengthPixy = 0.28;
float focalLengthHeight = 232.50;
float focalLengthWidth = 267.15;


int pixelsWidth;   //read by the camera
int pixelsHeight; //read by the camera
float distanceWidth;   //calculated distance based on the width of the object
float distanceHeight;  //calculated distance based on the height of the object 
float widthOfObject = 13.7; //inches (3.75 inches) real size of your object
float heightOfObject = 4; //inches (2.5 inches) real size of your object
float distance_ = 30;
// This is the main Pixy oject 
Pixy2 pixy;
 
void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial.print("Starting...\n");

  pixy.init();
  pixy.ccc.getBlocks();/*
        pixelsWidth = pixy.ccc.blocks[0].m_width;
      pixelsHeight = pixy.ccc.blocks[0].m_height;
  focalLengthHeight = (pixelsHeight * distance_) / heightOfObject;
  focalLengthWidth= (pixelsWidth * distance_) / widthOfObject;*/
}
 
void loop()
{ 
  int i; 
   // Serial.println(Serial3.read());

  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      
      pixelsWidth = pixy.ccc.blocks[i].m_width;
      pixelsHeight = pixy.ccc.blocks[i].m_height;
      distanceWidth = (focalLengthWidth * widthOfObject) /pixelsWidth;
      distanceHeight = (focalLengthHeight * heightOfObject) /pixelsHeight;
      Serial.print("This is distance Width : ");
      Serial.print(distanceWidth);
      Serial.print(" VS This is distance Height : ");
      Serial.println(distanceHeight);
      Serial.println(focalLengthHeight);
            Serial.println(focalLengthWidth);



      /*
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();*/
    }
  }  
}
