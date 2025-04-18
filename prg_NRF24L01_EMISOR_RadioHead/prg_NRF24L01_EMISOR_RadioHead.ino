#include <SPI.h>
#include <RH_NRF24.h>

// Crear una instància del controlador
RH_NRF24 nrf24;

void setup() {
  Serial.begin(9600);
  while (!Serial); // Esperar la connexió del port sèrie

  if (!nrf24.init()) {
    Serial.println("Inicialització fallida");
  }

  // Configurar el canal de comunicació (opcional)
  nrf24.setChannel(1);

  // Configurar la velocitat de dades i la potència de transmissió (opcional)
  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);
}

void loop() {
  const char *msg = "Hola món!";
  nrf24.send((uint8_t *)msg, strlen(msg));
  nrf24.waitPacketSent();
  Serial.println("Missatge enviat");
  delay(1000);
}
