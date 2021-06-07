# things-uno-ttnmapper
TTN Mapper node based on a Things Uno board and a NEO-6M GPS module.

![Things Uno TTN mapper node](https://github.com/brychanthomas/things-uno-ttnmapper/blob/main/mapper.jpg?raw=true)

The MAX_LAT, MIN_LAT, MAX_LONG, MIN_LONG, MAX_ALT and MIN_ALT constants should be adjusted for the range of coordinates and elevations that you expect to
record data for. The greater the range the lower the resolution but the greater the geographical area data can be recorded for.

It transmits a 9 byte payload, 3 bytes each for latitude and longitude, 2 bytes for altitude and 1 byte for HDOP. Low-Power is used and the GPS is put into sleep
mode to try and lengthen its battery life (I used a USB power pack).

It's only really a rough prototype made quickly, so there are many improvements that can be made in reducing the payload size and power consumption and so on.
