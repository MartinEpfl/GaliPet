/*
 /*
 # This Sample code is for testing the DC Motor Driver 2x15A_lite module.

 # Editor : Phoebe
 # Date   : 2012.11.6
 # Ver    : 0.1
 # Product: DC Motor Driver 2x15A_lite
 # SKU    : DRI0018

 # Description:
 # Drive 2 motors with this DC Motor Driver module

 # Hardwares:
 1. Arduino UNO
 2. DC Motor Driver 2x15A_lite
 3. DC motors x2

 #Steps:
 1.Connect the M1_PWM & M2_PWM to UNO digital 5 & 6
 2.Connect the M1_EN & M2_EN to UNO digital 4 & 7
 3.Connect +5V & GND to UNO 5V & GND

 # Function for current sense and diagnosis,if you want to use
 please connect the IS pins to Arduino
 Connect LA_IS and RA_IS to UNO digital 2 at the same time
 Connect LB_IS and RB_IS to UNO digital 3 at the same time
 */

int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 32;     //M1 Direction Control
int M2 = 33;     //M1 Direction Control
int counter=0;

void stop(void)                    //Stop
{
  digitalWrite(E1,0);
  digitalWrite(M1,LOW);
  digitalWrite(E2,0);
  digitalWrite(M2,LOW);
}
void advance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void back_off (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void current_sense()                  // current sense and diagnosis
{
  int val1=digitalRead(2);
  int val2=digitalRead(3);
  if(val1==HIGH || val2==HIGH){
    counter++;
    if(counter==3){
      counter=0;
      Serial.println("Warning");
    }
  }
}

void setup(void)
{
  Serial.begin(9600);      //Set Baud Rate
  int i;
  
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);
  Serial.println("Run keyboard control");
  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
}

void loop(void)
{
  /*
  static unsigned long timePoint = 0;    // current sense and diagnosis,if you want to use this
   if(millis() - timePoint > 1000){       //function,please show it & don't forget to connect the IS pins to Arduino
   current_sense();
   timePoint = millis();
   }
   */
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        Serial.print("Move forward");
        advance (255,255);   //move forward in max speed
        break;
      case 's'://Move Backward
        back_off (255,255);   //move back in max speed
        break;
      case 'a'://Turn Left
        turn_L (100,100);
        break;
      case 'd'://Turn Right
        turn_R (100,100);
        break;
      case 'z':
        Serial.println("Hello");
        break;
      case 'x':
        stop();
        break;
      }
    }
    else stop();
  }

}
