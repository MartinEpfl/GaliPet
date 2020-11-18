#include <Pixy2UART.h>
Pixy2UART pixy;

float focalLengthPixy = 0.28;
float focalLengthHeight = 232.50;
float focalLengthWidth = 267.15;


int pixelsWidth;   //read by the camera
int pixelsHeight; //read by the camera
float distanceWidth;   //calculated distance based on the width of the object
float distanceHeight;  //calculated distance based on the height of the object 
float averageDistance;
float widthOfObject = 13.7; //cms  real size of your object
float heightOfObject = 4; //cms real size of your object
const int sizeOfArray = 20;
double distancesAverage[sizeOfArray];
double tempDistanceAverage[sizeOfArray];
double value;
float distance_ = 30;


void setup()
{
  Serial.begin(19200);
  Serial.print("Starting...\n");
  
  pixy.init();
}

void loop()
{ 
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {

    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      pixelsWidth = pixy.ccc.blocks[i].m_width;
      pixelsHeight = pixy.ccc.blocks[i].m_height;
      distanceWidth = (focalLengthWidth * widthOfObject) /pixelsWidth;
      distanceHeight = (focalLengthHeight * heightOfObject) /pixelsHeight;
      averageDistance = (distanceWidth +distanceHeight)/2;
      for(int j = 1;j<sizeOfArray;j++){
        tempDistanceAverage[j] = distancesAverage[j-1];
      }
      tempDistanceAverage[0] = averageDistance;
      for(int j = 0;j<sizeOfArray;j++){
        distancesAverage[j] = tempDistanceAverage[j];
        value += distancesAverage[j];
      }
      value/=sizeOfArray;
      Serial.print("This is the value: ");
      Serial.println(value);

    }
  }  
}
