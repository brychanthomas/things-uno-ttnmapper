function Decoder(bytes, port) {
  
  var MAX_LAT = 59.0
  var MIN_LAT = 49.0
  var MAX_LONG = 2.0
  var MIN_LONG = -9.0
  var MAX_ALT = 1350.0
  var MIN_ALT = -10.0
  
  var LAT_RANGE = MAX_LAT - MIN_LAT
  var LONG_RANGE = MAX_LONG - MIN_LONG
  var ALT_RANGE = MAX_ALT - MIN_ALT
  
  var decoded = {};
  
  decoded.latitude = (bytes[0] << 16) | (bytes[1] << 8) | (bytes[2]);
  decoded.latitude = (decoded.latitude * LAT_RANGE / 16777215) + MIN_LAT;
  decoded.latitude = Math.round(decoded.latitude*1000000) / 1000000; //round to 6 dp
  
  decoded.longitude = (bytes[3] << 16) | (bytes[4] << 8) | (bytes[5]);
  decoded.longitude = (decoded.longitude * LONG_RANGE / 16777215) + MIN_LONG;
  decoded.longitude = Math.round(decoded.longitude*1000000) / 1000000; //round to 6 dp
  
  decoded.altitude = (bytes[6] << 8) | bytes[7];
  decoded.altitude = (decoded.altitude * ALT_RANGE / 65535) + MIN_ALT;
  decoded.altitude = Math.round(decoded.altitude*10) / 10; //round to 1 dp
  
  decoded.hdop = bytes[8] / 10;

  return decoded;
}