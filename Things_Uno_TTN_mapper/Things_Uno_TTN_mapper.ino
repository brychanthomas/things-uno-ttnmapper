#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define GPS_TX 8
#define GPS_RX 9

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

char transmitBuffer[8];

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop() {
  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read()) && gps.location.isValid() && gps.hdop.hdop() < 20) {
      
    }
  }
}

void fillBuffer() {
  uint32_t latitude = gps.location.lat() + 90;
  uint32_t longitude = gps.location.lng() + 180;
  uint16_t altitude = gps.altitude.meters();
  uint8_t hdop = gps.hdop.hdop() * 10;
}
