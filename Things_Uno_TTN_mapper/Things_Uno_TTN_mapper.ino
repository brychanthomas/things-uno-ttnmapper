#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define GPS_TX 8
#define GPS_RX 9

#define MAX_LAT 59.0
#define MIN_LAT 49.0
#define MAX_LONG 2.0
#define MIN_LONG -9.0
#define MAX_ALT 1350.0
#define MIN_ALT -10.0

#define LAT_RANGE MAX_LAT - MIN_LAT
#define LONG_RANGE MAX_LONG - MIN_LONG
#define ALT_RANGE MAX_ALT - MIN_ALT

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

byte transmitBuffer[9];

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
  uint32_t latitude = 16777215 * (gps.location.lat() - MIN_LAT) / LAT_RANGE;
  uint32_t longitude = 16777215 * (gps.location.lng() - MIN_LONG) / LONG_RANGE;
  uint16_t altitude = 65536 * (gps.altitude.meters() - MIN_ALT) / ALT_RANGE;
  uint8_t hdop = round(gps.hdop.hdop() * 10);

  buffer[0] = (latitude >> 16) & 0xff;
  buffer[1] = (latitude >> 8) & 0xff;
  buffer[2] = latitude & 0xff;
  
  
  buffer[3] = (longitude >> 16) & 0xff;
  buffer[4] = (longitude >> 8) & 0xff;
  buffer[5] = longitude & 0xff;

  buffer[6] = (altitude >> 8) & 0xff;
  buffer[7] = altitude & 0xff;

  buffer[8] = hdop;
}

void printBuffer() {
  for (int i = 0; i<9; i++) {
    if (buffer[i] < 16) {Serial.print("0");}
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
}
