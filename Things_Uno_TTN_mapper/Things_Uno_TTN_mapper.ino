//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>

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

//TinyGPSPlus gps;
//SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

byte transmitBuffer[9];

double lat = 58.4738714413;
double lng = -5.43178314;
double alt = 167.32179832;
double hd = 3.1934567;

void setup() {
  Serial.begin(9600);
  //gpsSerial.begin(9600);
  delay(3000);
  fillBuffer();
  printBuffer();
  
}

void loop() {
//  if (gpsSerial.available() > 0) {
//    if (gps.encode(gpsSerial.read()) && gps.location.isValid() && gps.hdop.hdop() < 20) {
//      
//    }
//  }
  printBuffer();
  delay(10000);
}

void fillBuffer() {
  uint32_t latitude = 16777215 * (lat - MIN_LAT) / (LAT_RANGE);
  uint32_t longitude = 16777215 * (lng - MIN_LONG) / (LONG_RANGE);
  uint16_t altitude = 65535.0 * (alt - MIN_ALT) / (ALT_RANGE);
  Serial.println(altitude);
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

void printBuffer() {
  for (int i = 0; i<9; i++) {
    if (transmitBuffer[i] < 16) {Serial.print("0");}
    Serial.print(transmitBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
