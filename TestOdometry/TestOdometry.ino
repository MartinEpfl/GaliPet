int E1 = 4;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)


const byte encoder0pinA =  2;
const byte encoder0pinB = 3;
byte encoder0PinALast;
double duration = 0;
boolean directionRead = true;
unsigned long timeWaiting = 45000; //Tourne pendant une minute
unsigned long previousTime = millis();
unsigned long currentTime = millis();
unsigned long diffTime = currentTime - previousTime;
int index = 0;
int analogValues[7] = {105, 130, 155, 180, 205, 230,255};
double speedValues[7] = {0,0,0,0,0,0,0};
void setup() {
  Serial.begin(9600);
  encoderInit();
  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  
  digitalWrite(M1, HIGH);

 
}

void loop() {
  //Serial.println("Speed :");
  //Serial.println((1000*PI*12*duration/24)/75/(timeWaiting));
  duration = 0;
  if(index < 7){
      analogWrite(E1, analogValues[index]);
     delay(timeWaiting);    
     speedValues[index] = (1000*PI*12*duration/24)/75/(timeWaiting);
     Serial.print("Last speed : ");
     Serial.println(speedValues[index]);
     index++;
  }
  else if(index==7){
    analogWrite(E1, 0);
    Serial.println("DONE : Analog Values :");
    for(int i = 0; i < 7; i++)
    {
        Serial.println(analogValues[i]);
    }
    Serial.println("Speed values in cm/s");
    for(int i = 0; i < 7; i++)
    {
        Serial.println(speedValues[i]);
    }
    index++;
  }
 // delay(timeWaiting);
//  currentTime = millis();
 // diffTime = currentTime - previousTime;
 // previousTime = currentTime;
 // Serial.print("Diff of time is :");
 // Serial.println(diffTime);
}

void encoderInit(){
  directionRead = true;
  pinMode(encoder0pinA, INPUT);
  pinMode(encoder0pinB, INPUT);
  encoder0PinALast = digitalRead(encoder0pinA);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), wheelSpeed, CHANGE);

}

void wheelSpeed(){

  int aState = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && aState == HIGH){
    int val = digitalRead(encoder0pinB);
    if(val == LOW && directionRead){
      directionRead = false;
      }
    else if(val==HIGH && !directionRead){
      directionRead = true;
      }
  }
  encoder0PinALast = aState;
  if(!directionRead) duration++;
  else duration--;
}
