
// ARDUINO  Y  PUENTE  H , Nº4  2  MOTORES
// CONTROL 2  MOTORES  CON  VOLTIMETRO Y 2 POTENCIOMETROS
const int potencioPin1 = A0; // Potentiometer connected to A0
const int potencioPin2 = A1; // Potentiometer connected to A1

const int entrada1 = 3;  // PWM pin connected to MOSFET Gate (D3)
const int entrada2 = 5;  // PWM pin DIR1
const int entrada3 = 9;  // conected  TO MOSFET
const int entrada4 = 10; //  conected DIR2

int potenValue1 = 0;
int potenValue2 = 0;
int pwmValue1 = 0;
int pwmValue2 = 0;
int pwmValue3 = 0;
int pwmValue4 = 0;

// VARIABLES DE VOLTIMETRO
const int analogInPin = A2; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 6; // Analog output pin that the LED is attached to

int sensorValue = 0; // value read from the pot
int outputValue = 0; // value output to the PWM (analog out)

void setup() {
  pinMode(entrada1, OUTPUT);
  pinMode(entrada2, OUTPUT);
  pinMode(entrada3, OUTPUT);
  pinMode(entrada4, OUTPUT);
  Serial.begin(9600); // Initialize serial communication at 9600 baud
}

void loop() { 
   // MOTORES
   // Read the potentiometer value  de  A0  y  A1  de   of 2  pins(0 to 1023)
  potenValue1 = analogRead(potencioPin1);
  potenValue2 = analogRead(potencioPin2);

  // inicialitzem valors digitals del voltatge
  pwmValue1 = 0;
  pwmValue2 = 0;
  pwmValue3 = 0;
  pwmValue4 = 0;

  // anem cap endavant motor dreta
  if(potenValue1 > 400) {
    // Map the potentiometer value to PWM range (0 to 190  forward motor 1 y 2)
    pwmValue1 = map(potenValue1, 400, 1023, 0, 250); // PRIMER  MOTOR MOSET
    pwmValue2 = 0;
  }
  // anem cap endavant motor esquerra
  if(potenValue2 > 400) {
    // Map the potentiometer value to PWM range (0 to 190  forward motor 1 y 2)
    pwmValue3 = map(potenValue2, 400, 1023, 0, 250);  // SEHUNDO MOTOR  MOSET
    pwmValue4 = 0;
  }

  // anem cap endarrere motor dreta
  if(potenValue1 <= 400) {
    //  Map  potenciometer value  PWM range  191 to 250 backward  motor 1  y 2 )
    pwmValue1 = map(potenValue1, 400, 0, 0, 100);   // PTIMER  MOTOR  DIR
    pwmValue2 = 250;
  }

  // anem cap endarrere motor esquerra
  if(potenValue2 <= 400) {
    //  Map  potenciometer value  PWM range  191 to 250 backward  motor 1  y 2 )
    pwmValue3 = map(potenValue2, 400, 0, 0, 100);   // SEGUNDO  MOTOR  DIR 
    pwmValue4 = 250;   
  }

  
  
  
  // Write the PWM value to the MOSFET gate to control motor speed
  analogWrite(entrada1, pwmValue1);  // PIN 3, poten 1
  analogWrite(entrada2, pwmValue2);  // PIN 5, poten 1
  analogWrite(entrada3, pwmValue3);  // PIN 9, poten 2
  analogWrite(entrada4, pwmValue4);  // PIN 10, poten 2
  Serial.println("entrada1: "); Serial.println(entrada1);
  Serial.println(pwmValue1);
  Serial.println(potenValue1);  
  Serial.println("entrada2: ");Serial.println(entrada2);
  Serial.println(pwmValue2);
  Serial.println("entrada3: ");Serial.println(entrada3);
  Serial.println(pwmValue3);
  Serial.println(potenValue2);
    Serial.println("entrada4: ");Serial.println(entrada4);
  Serial.println(pwmValue4);
  // SENSOR  VOLTAJE
  // read the analog in value:
 sensorValue = analogRead(analogInPin); 
 // map it to the range of the analog out:
 outputValue = map(sensorValue, 0, 1023, 0, 28); //  VALOR  FINAL
 // change the analog out value:
 analogWrite(analogOutPin, outputValue);
 Serial.print("voltaje: ");
 Serial.println (outputValue);
 delay(1500);

}

  /*
  digitalWrite(entrada1, HIGH);//  IR  HACIA   DELANTE
  digitalWrite(entrada2, LOW);
  digitalWrite(entrada3, HIGH);
  digitalWrite(entrada4, LOW);
  delay(2000);
  digitalWrite(entrada1, LOW);//   GIRAR  A  IZQUIERDA
  digitalWrite(entrada2, HIGH);
  digitalWrite(entrada3, HIGH);
  digitalWrite(entrada4, LOW);
  delay(2000);
  digitalWrite(entrada1, HIGH);//  GIRAR  A  DERECHA
  digitalWrite(entrada2, LOW);
  digitalWrite(entrada3, LOW);
  digitalWrite(entrada4, HIGH);
  delay(2000);
  digitalWrite(entrada1, LOW);//  ATRAS
  digitalWrite(entrada2, HIGH);
  digitalWrite(entrada3, LOW);
  digitalWrite(entrada4, HIGH);
  delay(10000);
*/
