#include <SoftwareSerial.h>

// Defineix els pins que uses per al mòdul LoRa
#define LORA_RX 10  // Arduino rep des del TX del LoRa
#define LORA_TX 11  // Arduino envia cap al RX del LoRa

SoftwareSerial lora(LORA_RX, LORA_TX); // RX, TX

void setup() {
  Serial.begin(9600);      // Comunicació amb el PC (monitor sèrie)
  lora.begin(9600);        // Comunicació amb el mòdul LoRa

  Serial.println("LoRa READY! Escriu per enviar:");
}

void loop() {
  // Si reps alguna cosa del monitor sèrie, envia-ho al LoRa
  if (Serial.available()) {
    char c = Serial.read();
    lora.write(c);       // Envia byte a LoRa
  }

  // Si reps dades del LoRa, mostra-les al monitor sèrie
  if (lora.available()) {
    char c = lora.read();
    Serial.write(c);     // Mostra byte rebut
  }
}
