#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TheThingsNetwork.h>

/**
 * todo:
 * - test gpsOn and gpsOff
 * - add radio code
 * - add low power code
 */

#define GPS_TX 8
#define GPS_RX 9

#define MAX_LAT 53.5
#define MIN_LAT 51.0
#define MAX_LONG -2.0
#define MIN_LONG -5.5
#define MAX_ALT 1100.0
#define MIN_ALT -10.0

#define LAT_RANGE MAX_LAT - MIN_LAT
#define LONG_RANGE MAX_LONG - MIN_LONG
#define ALT_RANGE MAX_ALT - MIN_ALT

#define loraSerial Serial1

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

const char *devAddr = "000000";
const char *nwkSKey = "00000000000000000000000000000000";
const char *appSKey = "00000000000000000000000000000000";

TheThingsNetwork ttn(loraSerial, Serial, TTN_FP_EU868);

byte transmitBuffer[9];

void setup() {
  Serial.begin(9600);
  //gpsSerial.begin(9600);
  loraSerial.begin(57600);
  delay(3000);
  ttn.personalize(devAddr, nwkSKey, appSKey);
  ttn.showStatus();
}

void loop() {
  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read()) && gps.location.isValid() && gps.hdop.hdop() < 20) {
      fillBuffer(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.hdop.hdop());
      printBuffer();
      transmit();
    }
  }
}

void fillBuffer(double lat, double lng, double alt, double hd) {
  uint32_t latitude = 16777215 * (lat - MIN_LAT) / (LAT_RANGE);
  uint32_t longitude = 16777215 * (lng - MIN_LONG) / (LONG_RANGE);
  uint16_t altitude = 65535 * (alt - MIN_ALT) / (ALT_RANGE);
  uint8_t hdop = round(hd * 10);

  transmitBuffer[0] = (latitude >> 16) & 0xff;
  transmitBuffer[1] = (latitude >> 8) & 0xff;
  transmitBuffer[2] = latitude & 0xff;
  
  
  transmitBuffer[3] = (longitude >> 16) & 0xff;
  transmitBuffer[4] = (longitude >> 8) & 0xff;
  transmitBuffer[5] = longitude & 0xff;

  transmitBuffer[6] = (altitude >> 8) & 0xff;
  transmitBuffer[7] = altitude & 0xff;

  transmitBuffer[8] = hdop;
}

void transmit() {
  ttn.sendBytes(transmitBuffer, sizeof(transmitBuffer));
}

void printBuffer() {
  for (int i = 0; i<9; i++) {
    if (transmitBuffer[i] < 16) {Serial.print("0");}
    Serial.print(transmitBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void gpsOff() {
  uint8_t ubx[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  for(int i=0; i<16; i++) {
    gpsSerial.write(ubx[i]);
  }
}

void gpsOn() {
  gpsSerial.write(0x01);
}
