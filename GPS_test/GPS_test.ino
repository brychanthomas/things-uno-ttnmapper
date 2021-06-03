/**
 * Sketch for testing NEO-6M. Outputs data to serial.
 * Prints longitude, latitude, altitude, HDOP, satellite count.
 * Can use http://epsg.io/map to check coordinates.
 */

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define GPS_TX 8
#define GPS_RX 9

TinyGPSPlus gps;

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  delay(1000);
  gpsOn();

}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid() && gps.hdop.hdop() < 50 && gps.location.isUpdated()) {
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
        Serial.println("TURNING OFF...");
        gpsOff();
        delay(5000);
        Serial.println("TURNING ON...");
        gpsOn();
      } else {
        Serial.println("Invalid location!");
      }
    }
  }
}

// normally (when trying to obtain fix) the NEO-6M module seems to use around 50mA
// but in this mode it uses around 11mA
void gpsOff() {
  uint8_t ubx[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  for(int i=0; i<16; i++) {
    gpsSerial.write(ubx[i]);
  }
}

void gpsOn() {
  gpsSerial.write(0x01);
}
