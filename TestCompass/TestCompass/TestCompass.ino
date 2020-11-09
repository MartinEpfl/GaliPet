    
/*
Gy-26 - Compass.
Criado por Igor Araujo - www.igoraraujo.eng.br - 2012
*/

char valorbyte[8];
int graus = 0;
int contador = 0;
byte valor = 0;

void setup() {

  Serial.begin(9600);
  Serial2.begin(9600);

}

void loop() {

  leitura();

}

void leitura() {

  valor = 0;

  Serial2.write(0x31);
  while (valor == 0) {
    if (Serial2.available()) {
      valorbyte[contador] = Serial2.read();
      contador = (contador + 1) % 8;
      if (contador == 0) {
        graus = (valorbyte[2] - 48) * 100 + (valorbyte[3] - 48) * 10 + (valorbyte[4] - 48);
        valor = 1;
      }
    }
  }
  Serial.println(graus);
  delay(300);
}
