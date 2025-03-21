//  GPS  IR  A  DESTINO

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define RXPin 8
#define TXPin 7
#define GPSBaud 9600
#define ConsoleBaud 115200
const int entrada1 = 3;  // PWM pin connected to MOSFET Gate (D3)
const int entrada2 = 5;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// The TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastUpdateTime = 0;

//  PREMIA  DE  MAR  U  OTRA  POBLACION  EN  GRADOS  DECIMALES
//  RELLENA  LOS  DATOS  DE  DESTINO  LNG  Y  LAT  QUE  DESEAS:
#define des_LNG  2.365253 
#define des_LAT 41.491594 


// Variables per guardar les coordenades
float latitudInicial = 0.0;
float longitudInicial = 0.0;
bool coordenadesGuardades = false;

/* This example shows a basic framework for how you might
   use course and distance to guide a person (or a drone)
   to a destination.  This destination is the Premia de Mar
   Change it as required.

   The easiest way to get the lat/long coordinate is to
   right-click the destination in Google Maps (maps.google.com),
   and choose "What's here?".  This puts the exact values in the
   search box.
*/

void setup()
{
  Serial.begin(ConsoleBaud);
  ss.begin(GPSBaud);

}

void loop()
{
  // If any characters have arrived from the GPS,
  // send them to the TinyGPS++ object
  while (ss.available() > 0)
    gps.encode(ss.read());

    // Every 5 seconds, do an update.
    if (millis() - lastUpdateTime >= 5000)
    {
      lastUpdateTime = millis();
      Serial.println();

      if (gps.location.isValid() && !coordenadesGuardades) {
                    latitudInicial = gps.location.lat();
                    longitudInicial = gps.location.lng();
                    coordenadesGuardades = true;
                    Serial.println("Coordenades inicials guardades!");
       }

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