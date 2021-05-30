/**
 * Sketch for testing NEO-6M. Outputs data to serial.
 * Prints longitude, latitude, altitude, HDOP, satellite count.
 * Can use http://epsg.io/map to check coordinates.
 */

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define TX 2
#define RX 3

TinyGPSPlus gps;

SoftwareSerial gpsSerial(9, 8);

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  delay(1000);

}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.println("------------------");
        Serial.print("Long: ");
        Serial.println(gps.location.lng(), 8);
        Serial.print("Lat: ");
        Serial.println(gps.location.lat(), 8);
        Serial.print("Alt: ");
        Serial.println(gps.altitude.meters());
        Serial.println("HDOP: ");
        Serial.println(gps.hdop.hdop());
        Serial.print("Sats: ");
        Serial.println(gps.satellites.value());
      } else {
        Serial.println("Invalid location!");
      }
    }
  }

//  if (millis() > 5000 && gps.charsProcessed() < 10)
//  {
//    Serial.println("No GPS detected");
//    while(true);
//  }

}
