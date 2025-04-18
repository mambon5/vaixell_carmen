
/*
	Capitulo 46 de Arduino desde cero en Español.
	Programa que mediante dos pulsadores en un modulo emisor RF de 433 Mhz
	permite enviar el caracter 1 o 2.
	Requiere instalar libreria RadioHead.h

	Autor: bitwiseAr  

*/


/// Programa 2 lado Emisor ////

#include <RH_ASK.h>		// incluye libreria RadioHead.h
#include <SPI.h> 		// incluye libreria SPI necesaria por RadioHead.h
 
RH_ASK rf_driver;		// crea objeto para modulacion por ASK

#define PULSADOR1 1		// reemplaza ocurrencia de PULSADOR1 por el numero 2
#define PULSADOR2 0		// reemplaza ocurrencia de PULSADOR2 por el numero 3
 char msg = '0';
 #define GPSBaud 9600
#define ConsoleBaud 115200

void setup(){
     pinMode(PULSADOR1, INPUT_PULLUP);	// pin 2 como entrada con resistencia de pull-up
     pinMode(PULSADOR2, INPUT_PULLUP);	// pin 3 como entrada con resistencia de pull-up
    rf_driver.init();   // inicializa objeto con valores por defecto
    Serial.begin(ConsoleBaud);
}
 
void loop(){
    
    // rebre un caràcter desde la consola
    char charEnviat = Serial.read();  // Llegeix un caràcter
  //  Serial.print("Instruccio general: ");
    if(charEnviat == '1' || charEnviat == '2' || charEnviat == '3' || charEnviat == '4'  ) {
       Serial.print("Canvi en Instruccio general: ");
      msg = charEnviat;
      Serial.println(msg);  // Mostra el caràcter rebut
    }
    //Serial.println(msg);  // Mostra el caràcter rebut
    
    if (digitalRead( msg ))

    {	
    // carga numero 1 en mensaje a enviar
    rf_driver.send((uint8_t *)&msg, 1); // envia 1 byte
    rf_driver.waitPacketSent();			// espera al envio correcto del mensaje
    Serial.println("missatge enviat"); 
    Serial.println(msg); 
   		// espera al envio correcto del mensaje
    }

    delay(200);					// demora de 200 mseg.
}
