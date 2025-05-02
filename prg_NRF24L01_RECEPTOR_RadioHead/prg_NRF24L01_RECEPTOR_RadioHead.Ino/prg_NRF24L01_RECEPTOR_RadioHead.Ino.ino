#include <SPI.h>
#include <RH_NRF24.h>

// Crear una instància del controlador
RH_NRF24 nrf24(9, 10); // CE = 9, CSN = 10



void setup() {
  Serial.begin(9600);
  while (!Serial); // Esperar la connexió del port sèrie

  if (!nrf24.init()) {
    Serial.println("Inicialització fallida");
  }

  // Configurar el canal de comunicació (ha de coincidir amb l'emissor)
  nrf24.setChannel(1);

  // Configurar la velocitat de dades i la potència de transmissió (ha de coincidir amb l'emissor)
  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);
}

void loop() {
  if (nrf24.available()) {
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (nrf24.recv(buf, &len)) {
      Serial.print("Missatge rebut: ");
      Serial.println((char*)buf);
    } else {
      Serial.println("Error en la recepció");
    }
  }
}
