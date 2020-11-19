#include <PID_v1.h>


int E1 = 6;     //M1 Speed Control (PWM)
int M1 = 27;     //M1 Direction Control (Digital)


// Définition des variables pour l'encodeur
const byte encoder0pinA =  3;
const byte encoder0pinB = 4;
byte encoder0PinALast;
double ticks = 0;
boolean directionRead = true;
unsigned long timeWaiting = 100; //Tourne pendant un temps spécifié
unsigned long previousTime = millis();
unsigned long currentTime = millis();
unsigned long diffTime = currentTime - previousTime; // temps calculé par l'arduino


// Définition des variables pour le PID
double motorspeed = 0;
double setspeed = 15;
double pwmOut = 0;
PID myPID(&motorspeed, &pwmOut, &setspeed,5.1,2,0.005, DIRECT);


void setup() {
  Serial.begin(9600);
  encoderInit();
  pinMode(E1, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M1, HIGH);
  analogWrite(E1, pwmOut);
  
  motorspeed = 0;

  // PID on
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
}

void loop() {
  //Serial.println((1000*PI*12*ticks/24)/75/(timeWaiting));
  

  currentTime = millis();
  diffTime = currentTime - previousTime;
  previousTime = currentTime;
  motorspeed = (PI*12)*(ticks/(24*75))*(1000/diffTime);
  Serial.print(motorspeed);
  Serial.print("  ");
  Serial.println(pwmOut);

//  Serial.print("  ");
 // Serial.println(currentTime);    
  ticks = 0;
  delay(20);
  myPID.Compute();
  analogWrite(E1, pwmOut);
  

  


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
  if(!directionRead) ticks++;
  else ticks--;
}
