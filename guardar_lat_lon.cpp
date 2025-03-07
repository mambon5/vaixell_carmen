#include <SD.h>
#include <SPI.h>

const int chipSelect = 10;  // Pin del CS del mòdul SD

void setup() {
    Serial.begin(9600);
    
    if (!SD.begin(chipSelect)) {
        Serial.println("Error inicialitzant la SD");
        return;
    }
    
    Serial.println("SD inicialitzada");

    // Exemple de coordenades del punt inicial
    float lat = 41.3851;  // Exemple: Barcelona
    float lon = 2.1734;
    
    File dataFile = SD.open("coordenades.txt", FILE_WRITE);
    
    if (dataFile) {
        dataFile.print(lat, 6);
        dataFile.print(",");
        dataFile.println(lon, 6);
        dataFile.close();
        Serial.println("Coordenades guardades!");
    } else {
        Serial.println("Error obrint el fitxer");
    }
}

void loop() {
}
