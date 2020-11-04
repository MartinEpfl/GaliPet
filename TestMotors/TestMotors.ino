

int E1 = 4;     //M1 Speed Control
int M1 = 33;     //M1 Direction Control
int counter=0;

void stop(void)                    //Stop
{
  digitalWrite(E1,0);
  digitalWrite(M1,LOW);

}
void advance(char a)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);

}
void back_off (char a)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);

}


void setup(void)
{
  Serial.begin(115200);      //Set Baud Rate
  pinMode(E1, OUTPUT);
  Serial.println("Run keyboard control");
  //digitalWrite(E1,HIGH);
  pinMode(M1, OUTPUT);
}

void loop(void)
{
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        Serial.println("Move forward");
        advance (50);   //move forward in max speed
        break;
      case 's'://Move Backward
        back_off (5);   //move back in max speed
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
