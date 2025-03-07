#include <EEPROM.h>

void setup() {
    Serial.begin(9600);

    // Comprovem si ja hi ha una coordenada guardada
    float lat, lon;
    EEPROM.get(0, lat);
    EEPROM.get(sizeof(float), lon);

    if (isnan(lat) || isnan(lon) || lat == 0.0 || lon == 0.0) { 
        // Si no hi ha coordenades vàlides, guardem-ne unes de noves
        lat = 41.3851;  // Exemple: latitud de Barcelona
        lon = 2.1734;   // Exemple: longitud de Barcelona

        EEPROM.put(0, lat);
        EEPROM.put(sizeof(float), lon);

        Serial.println("Coordenades inicials guardades!");
    } else {
        Serial.println("Ja hi ha coordenades guardades.");
    }
}

void loop() {
}
