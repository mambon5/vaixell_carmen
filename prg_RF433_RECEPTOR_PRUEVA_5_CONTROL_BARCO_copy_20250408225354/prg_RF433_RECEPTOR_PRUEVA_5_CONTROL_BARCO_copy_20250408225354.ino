

// RF433 RECEPTOR CON LEDS
/*
  Autor: L_FLORIT+ ROMA_M, bitwiser
*/

//// Programa 2 lado RECEPTOR ////
     // ARDUINO  Y  PUENTE  H , Nº4  2  MOTORES
    // CONTROL 2  MOTORES  CON  VOLTIMETRO Y 2 POTENCIOMETROS

#include <RH_ASK.h>   // incluye libreria RadioHead.h
#include <SPI.h>    // incluye libreria SPI necesaria por RadioHead.h
RH_ASK rf_driver;   // crea objeto para modulacion por ASK

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//DEFINICION DEL  DESTINO:
#define des_LAT 41.73694 //  latitud  destino DELANTE DE  PUERTO  PALAMOS
#define des_LNG  3.27724 //  longitud destino  elegido  DELANTE DE  PUERTO  PALAMOS
     //   DESTINO.

//VARIABLES DE  LED
#define LEDAZUL 0   // reemplaza ocurrencia de LEDROJO por el numero 2
#define LEDVERDE 1    // reemplaza ocurrencia de LEDVERDE por el numero 1

//VARIABLES DE  GPS
#define RXPin 3
#define TXPin 4
#define GPSBaud 9600
#define ConsoleBaud 115200

// VARIABLES  DEL  MOTOR
const int entrada1 = 5;  // PWM pin connected to MOSFET. CABLE  AZUL
const int entrada2 = 6;  //  PWM pin DIR1
const int entrada3 = 9;  // conected  TO MOSFET. CABLE  AZUL
const int entrada4 = 8; //  conected DIR2
int pwmValue1 = 0;
int pwmValue2 = 0;
int pwmValue3 = 0;
int pwmValue4 = 0;


// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// The TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastUpdateTime = 0;

//LAS  4  LINEAS  INFERIORES  SON  PARA  ARDUINO DE  TIERRA
const int potencioPin1 = A0; // Potentiometer connected to A0  // LEO  EN  TIERRA
const int potencioPin2 = A1; // Potentiometer connected to A1  // LEO  EN  TIERRA
int potenValue1 = 0;  //EN  TIERRA  EN  REALIDAD
int potenValue2 = 0;  //EN  TIERRA  ES  LO QUE SE  ENVIA  POR  RF

// VARIABLES DE VOLTIMETRO DE  LAS  PILAS
const int analogInPin = A2; // Analog input pin that the potentiometer is attached to
const int analogOutPin = 6; // Analog output pin IS 6 PIN// JUNTO MOTOR 1  HACIA  ATRAS
int sensorValue = 0; // value read from the pin A2
int outputValue = 0; // value output to the PWM (analog out), pin 6

// Variables per guardar les coordenades
float latitudInicial = 0.0;
float longitudInicial = 0.0;
bool coordenadesGuardades = false;

// instruccio generalç
char InstGral = '0';
 
void setup()
{
    pinMode(LEDAZUL, OUTPUT); // pin 0 como salida  LED AZUL
    pinMode(LEDVERDE, OUTPUT);  // pin 1 como salida  VERDE

     // SALIDA  MOTORES
     pinMode(entrada1, OUTPUT);  
     pinMode(entrada2, OUTPUT);
     pinMode(entrada3, OUTPUT);
     pinMode(entrada4, OUTPUT);

    rf_driver.init();   // inicializa objeto con valores por defecto
    Serial.begin(9600); // Initialize serial communication at 9600 baud
    Serial.begin(ConsoleBaud);
    ss.begin(GPSBaud);


}
  

void loop() 
{   


  // SENSOR  VOLTAJE  LECTURA
 sensorValue = analogRead(analogInPin); 
 // map it to the range of the analog out:
 outputValue = map(sensorValue, 0, 1023, 0, 28); //  VALOR  FINAL
 // change the analog out value:
 analogWrite(analogOutPin, outputValue);
 Serial.print("voltaje: ");
 Serial.println (outputValue);
 delay(2000);

//  GUARDAR  COORDENADAS  DE  INICIO
if (gps.location.isValid() && !coordenadesGuardades) {
                    latitudInicial = gps.location.lat();
                    longitudInicial = gps.location.lng();
                    coordenadesGuardades = true;
                    Serial.println("Coordenades inicials guardades:");
                    Serial.println(latitudInicial,longitudInicial);
                    delay(5000);
 }

// SI  VOLTAJE  ES >=10 HARAS  UNA  DE  TRES  ORDENES:
  if(outputValue >=10 )   
 {
    uint8_t buf[1];     // espacio para mensaje recibido de 1 caracter
    uint8_t buflen = sizeof(buf); // longitud de buffer

    // rebre un caràcter desde la consola
    char charEnviat = Serial.read();  // Llegeix un caràcter
    Serial.print("Instruccio general: ");
    if(charEnviat == '1' || charEnviat == '2' || charEnviat == '3' || charEnviat == '4'  ) {
       Serial.print("Canvi en Instruccio general: ");
      InstGral = charEnviat;
    }
    Serial.println(InstGral);  // Mostra el caràcter rebut


    
    // if (rf_driver.recv(buf, &buflen)) // si se recibieron datos correctos
    if(InstGral != '\0')
    {
        if(  InstGral=='1')   // si el caracter es el numero 1
          {       
            Serial.println("hem entrat UN  CARACTER 1");
            digitalWrite(LEDVERDE, HIGH);		// enciende LED verde EN  PIN 1
            delay(5000);				// demora de 5 segundos
            digitalWrite(LEDVERDE, LOW);		// apaga LED verde
          // CONTROL MOTORES  CON  POTENCIOMETROS DESDE  TIERRA
          // Read the potentiometer value  de  A0  y  A1  de   of 2  pins(0 to 1023)
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
             //  Map  pot enciometer value  PWM range  191 to 250 backward  motor 1  y 2 )
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

        }
    if(InstGral=='2')   
            // si 2,  GPS  IR  A  DESTINO
        {  
              Serial.println("hem entrat en 2");
            digitalWrite(LEDAZUL, HIGH);		// enciende LED verde EN  PIN 1
            delay(5000);				// demora de 5 segundos
            digitalWrite(LEDAZUL, LOW);		// apaga LED verde 
        // If any characters have arrived from the GPS,
        // send them to the TinyGPS++ object
        while (ss.available() > 0)
          gps.encode(ss.read());

        // Every 5 seconds, do an update.
        if (millis() - lastUpdateTime >= 5000)
         {
          lastUpdateTime = millis();
          Serial.println();

          // Establish our current status
          double distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), des_LAT, des_LNG);
          double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), des_LAT, des_LNG);
          const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
          int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;

          // debug
          Serial.print("DEBUG: Course2Dest: ");
          Serial.print(courseToDestination);
          Serial.print("  CurCourse: ");
          Serial.print(gps.course.deg());
          Serial.print("  Dir2Dest: ");
          Serial.print(directionToDestination);
          Serial.print("  RelCourse: ");
          delay(10000);
          Serial.print(courseChangeNeeded);
          Serial.print("  CurSpd: ");
          Serial.println(gps.speed.kmph());
          delay(10000);

          float l = gps.location.lat();
          Serial.println();
          Serial.print("Lat: "); Serial.print(l,6); Serial.print("  Lon: "); Serial.println(gps.location.lng(),6);
          Serial.print("current Angle: "); Serial.println(atan2(gps.location.lat(), gps.location.lng())*180/M_PI);

          // Within 20 meters of destination?  We're here!
          if (distanceToDestination <= 20.0)
          {
            Serial.println("CONGRATULATIONS: You've arrived!");
            exit(1);
          }

          Serial.print("DISTANCE: ");
          Serial.print(distanceToDestination);
          Serial.println(" meters to go.");
          Serial.print("INSTRUCTION: ");

          // Standing still? Just indicate which direction to go.
          if (gps.speed.kmph() < 2.0)
          {
            Serial.print("Head ");
            Serial.print(directionToDestination);
            Serial.println(".");
            //return;
          }
                if (courseChangeNeeded >= 345 || courseChangeNeeded < 15)
            {
              Serial.println("Keep on straight ahead!");
              analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
              analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
            }
            else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345) 
            {
              Serial.println("Veer slightly to the left.");
              analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
              analogWrite(entrada2, 170);  // PIN 5, poten 1, MOTOR2
            }
            else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45)
            {
              Serial.println("Veer slightly to the right.");
              analogWrite(entrada1, 170);  // PIN 3, poten 1, MOTOR1
              analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
              }
            else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315)
            {
              Serial.println("Turn to the left.");
              analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
              analogWrite(entrada2, 140);  // PIN 5, poten 1, MOTOR2
            }
            else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105)
            {
              Serial.println("Turn to the right.");
              analogWrite(entrada1, 140);  // PIN 3, poten 1, MOTOR1
              analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
            }
            else
              {
              Serial.println("Turn completely around.");
              analogWrite(entrada1, 100);  // PIN 3, poten 1, MOTOR1
              analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
              }
        
            
         }
      
        }

        if(InstGral=='3')   // si el caracter es el numero 3
    {  
        // VUELTA A  PUNTO  DE  ORIGEN CON  GPS  POR  MANADATO DE TIERRA
        // Every 5 seconds, do an update.
    if (millis() - lastUpdateTime >= 5000)
    {
      lastUpdateTime = millis();
      Serial.println();

      // if (gps.location.isValid() && !coordenadesGuardades) {
      //               latitudInicial = gps.location.lat();
      //               longitudInicial = gps.location.lng();
      //               coordenadesGuardades = true;
      //               Serial.println("Coordenades inicials guardades!");
      //  }

      // Establish our current status
      double distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),latitudInicial, longitudInicial);
      double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), latitudInicial, longitudInicial);
      const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
      int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;

      // debug
      Serial.print("DEBUG: Course2Dest: ");
      Serial.print(courseToDestination);
      Serial.print("  CurCourse: ");
      Serial.print(gps.course.deg());
      Serial.print("  Dir2Dest: ");
      Serial.print(directionToDestination);
      Serial.print("  RelCourse: ");
      Serial.print(courseChangeNeeded);
      Serial.print("  CurSpd: ");
      Serial.println(gps.speed.kmph());

      float l = gps.location.lat();
      Serial.println();
      Serial.print("Lat: "); Serial.print(l,6); Serial.print("  Lon: "); Serial.println(gps.location.lng());
      Serial.print("current Angle: "); Serial.println(atan2(gps.location.lat(), gps.location.lng())*180/M_PI);

      // Within 20 meters of destination?  We're here!
      if (distanceToDestination <= 20.0)
      {
        Serial.println("CONGRATULATIONS: You've arrived!");
        exit(1);
      }

      Serial.print("DISTANCE: ");
      Serial.print(distanceToDestination);
      Serial.println(" meters to go.");
      Serial.print("INSTRUCTION: ");

      // Standing still? Just indicate which direction to go.
      if (gps.speed.kmph() < 2.0)
      {
        Serial.print("Head ");
        Serial.print(directionToDestination);
        Serial.println(".");
        //return;
      }

      if (courseChangeNeeded >= 345 || courseChangeNeeded < 15)
      {
        Serial.println("Keep on straight ahead!");
        analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
      }
      else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345) 
      {
        Serial.println("Veer slightly to the left.");
        analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 170);  // PIN 5, poten 1, MOTOR2
      }
      else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45)
      {
        Serial.println("Veer slightly to the right.");
        analogWrite(entrada1, 170);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
        }
      else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315)
      {
        Serial.println("Turn to the left.");
        analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 140);  // PIN 5, poten 1, MOTOR2
      }
      else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105)
      {
        Serial.println("Turn to the right.");
        analogWrite(entrada1, 140);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
       }
       else
        {
        Serial.println("Turn completely around.");
        analogWrite(entrada1, 100);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
        }
    }
        }
           
        else if((char)buf[0]=='4')  // si el caracter es el numero 4
         {
                digitalWrite(LEDVERDE, HIGH);   // enciende LED verde
                delay(3000);        // demora de 5 segundos
                digitalWrite(LEDVERDE, LOW);    // apaga LED verde
            } 
      }              
  }
  else if(outputValue <=10)  // EL  VOLTAJE  ESTA  A UN  MEDIO  DE  SU  CAPACIDAD
  {  
        // VUELTA A  PUNTO  DE  ORIGEN CON  GPS  POR  MANADATO DE TIERRA
        // Every 5 seconds, do an update.
      if (millis() - lastUpdateTime >= 5000)
    {
      lastUpdateTime = millis();
      Serial.println();

      // if (gps.location.isValid() && !coordenadesGuardades) {
      //               latitudInicial = gps.location.lat();
      //               longitudInicial = gps.location.lng();
      //               coordenadesGuardades = true;
      //               Serial.println("Coordenades inicials guardades!");
      //  }

      // Establish our current status
      double distanceToDestination = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),latitudInicial, longitudInicial);
      double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), latitudInicial, longitudInicial);
      const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
      int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;

      // debug
      Serial.print("DEBUG: Course2Dest: ");
      Serial.print(courseToDestination);
      Serial.print("  CurCourse: ");
      Serial.print(gps.course.deg());
      Serial.print("  Dir2Dest: ");
      Serial.print(directionToDestination);
      Serial.print("  RelCourse: ");
      Serial.print(courseChangeNeeded);
      Serial.print("  CurSpd: ");
      Serial.println(gps.speed.kmph());

      float l = gps.location.lat();
      Serial.println();
      Serial.print("Lat: "); Serial.print(l,6); Serial.print("  Lon: "); Serial.println(gps.location.lng());
      Serial.print("current Angle: "); Serial.println(atan2(gps.location.lat(), gps.location.lng())*180/M_PI);

      // Within 20 meters of destination?  We're here!
      if (distanceToDestination <= 20.0)
      {
        Serial.println("CONGRATULATIONS: You've arrived!");
        exit(1);
      }

      Serial.print("DISTANCE: ");
      Serial.print(distanceToDestination);
      Serial.println(" meters to go.");
      Serial.print("INSTRUCTION: ");

      // Standing still? Just indicate which direction to go.
      if (gps.speed.kmph() < 2.0)
      {
        Serial.print("Head ");
        Serial.print(directionToDestination);
        Serial.println(".");
        //return;
      }

      if (courseChangeNeeded >= 345 || courseChangeNeeded < 15)
        {
        Serial.println("Keep on straight ahead!");
        analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
        }
      else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345) 
        {
        Serial.println("Veer slightly to the left.");
        analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 170);  // PIN 5, poten 1, MOTOR2
        }
      else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45)
        {
        Serial.println("Veer slightly to the right.");
        analogWrite(entrada1, 170);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
        }
      else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315)
        {
        Serial.println("Turn to the left.");
        analogWrite(entrada1, 200);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 140);  // PIN 5, poten 1, MOTOR2
        }
      else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105)
        {
        Serial.println("Turn to the right.");
        analogWrite(entrada1, 140);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
        }
      else
        {
        Serial.println("Turn completely around.");
        analogWrite(entrada1, 100);  // PIN 3, poten 1, MOTOR1
        analogWrite(entrada2, 200);  // PIN 5, poten 1, MOTOR2
        }
    }
      
  }    


}