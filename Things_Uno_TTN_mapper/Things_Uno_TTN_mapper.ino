#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TheThingsNetwork.h>
#include <LowPower.h>

//pins GPS module is connected to - beware, only some of the pins on the
//Leonardo (Things Uno) can be used for software serial!
#define GPS_TX 8
#define GPS_RX 9

//adjust these values for the area you want to survey
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

//ABP keys
const char *devAddr = "00000000";
const char *nwkSKey = "00000000000000000000000000000000";
const char *appSKey = "00000000000000000000000000000000";

TheThingsNetwork ttn(loraSerial, Serial, TTN_FP_EU868);

byte transmitBuffer[9];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //LED is continuously on while awaiting GPS fix
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  gpsSerial.begin(9600);
  loraSerial.begin(57600);
  delay(3000);
  ttn.personalize(devAddr, nwkSKey, appSKey);
  ttn.showStatus();
  gpsOn();
}

void loop() {
  if (gpsSerial.available() > 0) {
    //if new GPS location with sufficient accuracy obtained
    if (gps.encode(gpsSerial.read()) && gps.location.isValid() && gps.hdop.hdop() <= 2 && gps.satellites.value() > 0 && gps.location.isUpdated()) {
      //LED turns off when GPS fix obtained
      digitalWrite(LED_BUILTIN, LOW);
      fillBuffer(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.hdop.hdop());
      Serial.println(gps.hdop.hdop());
      Serial.println("Transmitting!");
      printBuffer();
      transmit();
      Serial.println("Starting sleep");
      sleep();
      Serial.println("Sleep ended!");
      //LED turns back on while awaiting GPS fix
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}

//store values in buffer ready for transmission
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

//transmit contents of buffer
void transmit() {
  ttn.sendBytes(transmitBuffer, sizeof(transmitBuffer));
  for (int i=0; i<5; i++) { //LED flashes five times after transmission
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

//print the contents of the buffer for debugging
void printBuffer() {
  for (int i = 0; i<9; i++) {
    if (transmitBuffer[i] < 16) {Serial.print("0");}
    Serial.print(transmitBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// go into low power for around 40 seconds
void sleep() {
  gpsOff(); //put GPS to sleep
  delay(500);
  ttn.sleep(40000); //put LoRa module to sleep
  delay(500);
  for (int i = 0; i<4; i++)   { //low power sleep for 40 seconds
    //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); - this doesn't seem to work
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
          TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, HIGH); //flash LED every 8 seconds when sleeping
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.println();
  delay(500);
  ttn.wake();
  delay(500);
  gpsOn();
  delay(500);
}

//put GPS to sleep. Consumes around 11mA in this mode and 50mA when turned on and
//trying to obtain fix
void gpsOff() {
  uint8_t ubx[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
  for(int i=0; i<16; i++) {
    gpsSerial.write(ubx[i]);
  }
}

//send a character to wake up the GPS
void gpsOn() {
  gpsSerial.write(0x01);
}
