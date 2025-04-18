  //RECEPTOR  NRF24L01.  NAYLAMP

 /*Si analizamos el código lo que hacemos es inicialmente configurar el módulo y luego e
nviar o leer los datos transmitidos por el módulo NRF24L01, la variable que se va a transmitir 
puede ser un solo dato o un array de datos como se lo está haciendo en este tutorial,
pero siempre la variable o vector que se va a recibir tiene que ser del mismo tamaño y tipo que la variable enviada, 
de lo contrario se perderán datos.*/ 

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
 
//Declaremos los pines CE y el CSN
#define CE_PIN 9
#define CSN_PIN 10
 
//Variable con la dirección del canal que se va a leer
byte direccion[5] ={'c','a','n','a','l'}; 

//creamos el objeto radio (NRF24L01)
RF24 radio(CE_PIN, CSN_PIN);

//vector para los datos recibidos
float datos[3];

void setup()
{
 //inicializamos el NRF24L01 
  radio.begin();
  //inicializamos el puerto serie
  Serial.begin(9600); 
  
  //Abrimos el canal de Lectura
  radio.openReadingPipe(1, direccion);
  
    //empezamos a escuchar por el canal
  radio.startListening();
 
}
 
void loop() {
 uint8_t numero_canal;
 //if ( radio.available(&numero_canal) )
 if ( radio.available() )
 {    
     //Leemos los datos y los guardamos en la variable datos[]
     radio.read(datos,sizeof(datos));
     
     //reportamos por el puerto serial los datos recibidos
     Serial.print("Dato0= " );
     Serial.print(datos[0]);
     Serial.print(" V, ");
     Serial.print("Dato1= " );
     Serial.print(datos[1]);
     Serial.print(" ms, ");
     Serial.print("Dato2= " );
     Serial.println(datos[2]);
 }
 else
 {
     Serial.println("No hay datos de radio disponibles");
 }
 delay(1000);
}


    

