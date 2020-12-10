/*
Gy-26 - Compass.
Criado por Igor Araujo - www.igoraraujo.eng.br - 2012
*/

char valorbyte[8];
int graus = 0;
int contador = 0;
byte valor = 0;
double diff = 0;
double angle = 0;
void setup() {  

  Serial.begin(9600);
  Serial3.begin(9600);
   leitura();

  diff = PI/4 - angle;
}

void loop() {

  leitura();
  Serial.println(angle);

}

void leitura() {

  valor = 0;

  Serial3.write(0x31);
  while (valor == 0) {
    if (Serial3.available()) {
      valorbyte[contador] = Serial3.read();
      contador = (contador + 1) % 8;
      if (contador == 0) {
        graus = (valorbyte[2] - 48) * 100 + (valorbyte[3] - 48) * 10 + (valorbyte[4] - 48);
        valor = 1;
      }
    }
  }
  Serial.print("En degree : ");
  Serial.println(graus);
  angle = 2*PI*(360-graus)/360 + diff;
  delay(300);
}
