
// RF433 RECEPTOR CON LEDS
/*
  Autor: L_FLORIT+ ROMA_M, bitwiseAr
*/

//// Programa 2 lado RECEPTOR ////
     // ARDUINO  Y  PUENTE  H , Nº4  2  MOTORES
    // CONTROL 2  MOTORES  CON  VOLTIMETRO Y 2 POTENCIOMETROS

#include <RH_ASK.h>   // incluye libreria RadioHead.h
#include <SPI.h>    // incluye libreria SPI necesaria por RadioHead.h
RH_ASK rf_driver;   // crea objeto para modulacion por ASK

//VARIABLES DE  LED
#define LEDROJO 2   // reemplaza ocurrencia de LEDROJO por el numero 2
#define LEDVERDE 1    // reemplaza ocurrencia de LEDVERDE por el numero 3

// VARIABLES  DEL  MOTOR
const int entrada1 = 3;  // PWM pin connected to MOSFET Gate (D3)
const int entrada2 = 5;  // PWM pin DIR1
const int entrada3 = 9;  // conected  TO MOSFET
const int entrada4 = 10; //  conected DIR2
int pwmValue1 = 0;
int pwmValue2 = 0;
int pwmValue3 = 0;
int pwmValue4 = 0;

//LAS  4  LINEAS  INFERIORES  SON  PARA  ARDUINO DE  TIERRA
//const int potencioPin1 = A0; // Potentiometer connected to A0  // LEO  EN  TIERRA
//const int potencioPin2 = A1; // Potentiometer connected to A1  // LEO  EN  TIERRA
int potenValue1 = 0;  //EN  TIERRA  EN  REALIDAD
int potenValue2 = 0;  //EN  TIERRA  ES  LO QUE SE  ENVIA  POR  RF

// VARIABLES DE VOLTIMETRO DE  LAS  PILAS
const int analogInPin = A2; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 6; // Analog output pin that the LED is attached to
int sensorValue = 0; // value read from the pin A2
int outputValue = 0; // value output to the PWM (analog out), pin 6


 
void setup(){
    pinMode(LEDROJO, OUTPUT); // pin 2 como salida  LED
    pinMode(LEDVERDE, OUTPUT);  // pin 1 como salida

     // SALIDA  MOTORES
     pinMode(entrada1, OUTPUT);  
     pinMode(entrada2, OUTPUT);
     pinMode(entrada3, OUTPUT);
     pinMode(entrada4, OUTPUT);

    rf_driver.init();   // inicializa objeto con valores por defecto
    Serial.begin(9600); // Initialize serial communication at 9600 baud
}
  

void loop() {   
  
  // SENSOR  VOLTAJE
  // read the analog in value:
 sensorValue = analogRead(analogInPin); 
 // map it to the range of the analog out:
 outputValue = map(sensorValue, 0, 1023, 0, 28); //  VALOR  FINAL
 // change the analog out value:
 analogWrite(analogOutPin, outputValue);
 Serial.print("voltaje: ");
 Serial.println (outputValue);
 delay(2000);


// guardi latitiud i longitud actuals



  if(outputValue >=9 )  {

     uint8_t buf[1];     // espacio para mensaje recibido de 1 caracter
    uint8_t buflen = sizeof(buf); // longitud de buffer
    
    if (rf_driver.recv(buf, &buflen)) // si se recibieron datos correctos
    {
              if((char)buf[0]=='1')   // si el caracter es el numero 1
                {
                    
          // CONTROL MOTORES  CON  POTENCIOMETROS DESDE  TIERRA
          // Read the potentiometer value  de  A0  y  A1  de   of 2  pins(0 to 1023)
          //potenValue1 = analogRead(potencioPin1);  //  LEO  POTENCIOMETRO  EN  TIERRA
          //potenValue2 = analogRead(potencioPin2);  // LEO  POTENCIOMETRO 2 EN  TIERRA
          // Map the potentiometer value to PWM range (0 to 190  forward motor 1 y 2)
          pwmValue1 = map(potenValue1, 200, 1023, 0, 230); // PRIMER  MOTOR MOSET
          pwmValue2 = map(potenValue2, 200, 1023, 0, 230);  // SEHUNDO MOTOR  MOSET
          //  Map  potenciometer value  PWM range  191 to 250 backward  motor 1  y 2 )
          pwmValue3 = map(potenValue1, 0, 199, 250, 230);   // PTIMER  MOTOR  DIR
          pwmValue4 = map(potenValue2, 0, 199, 250, 230);   // SEGUNDO  MOTOR  DIR
          
          // Write the PWM value to the MOSFET gate to control motor speed
          analogWrite(entrada1, pwmValue1);  // PIN 3, poten 1
          analogWrite(entrada2, pwmValue3);  // PIN 5, poten 1
          analogWrite(entrada3, pwmValue2);  // PIN 9, poten 2
          //analogWrite(entrada4, pwmValue4);  // PIN 10, poten 2 DE MOMENTO  NO  CONECTO

        }
        if((char)buf[0]=='2')   // si el caracter es el numero 2
                {     
          // CONTROL  MOTORES  SEGÚN  DESTINO  DE  GPS MARCADO  PREVIAMENTE
        }

        if((char)buf[0]=='3')   // si el caracter es el numero 3
                {     
          // VUELTA A  PUNTO  DE  ORIGEN CON  GPS  POR  MANADATO DE TIERRA
        }
           
        else if((char)buf[0]=='4')  // si el caracter es el numero 4
        {
            digitalWrite(LEDVERDE, HIGH);   // enciende LED verde
            delay(3000);        // demora de 5 segundos
            digitalWrite(LEDVERDE, LOW);    // apaga LED verde
        } 
    }              
  }
  else if(outputValue <=9)  // EL  VOLTAJE  ESTA  A UN  TERCIO  DE  SU  CAPACIDAD
      {
        //MEDIA VUELTA  HACIA ORIGEN POR  BAJO  VOLTAJE
          
      }    


}
