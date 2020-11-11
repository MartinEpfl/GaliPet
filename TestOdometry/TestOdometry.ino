int E1 = 4;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)


const byte encoder0pinA =  2;
const byte encoder0pinB = 3;
byte encoder0PinALast;
double duration;
boolean directionRead = true;
int timeWaiting = 1000;
unsigned long previousTime = millis();
unsigned long currentTime = millis();
unsigned long diffTime = currentTime - previousTime;
void setup() {
  Serial.begin(9600);
  encoderInit();

  pinMode(M1, OUTPUT);
  digitalWrite(M1, HIGH);
  analogWrite(E1, 100);
 
}

void loop() {
  Serial.println("Pulse :");
  Serial.println((1000*PI*12*duration/24)/75/diffTime);
  duration = 0;
  delay(timeWaiting);
  currentTime = millis();
  diffTime = currentTime - previousTime;
  previousTime = currentTime;
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
