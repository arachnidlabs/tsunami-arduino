/** 
 * WaveformMeasurement.ino
 * Reports basic waveform measurements over USB serial.
 */

#include <SPI.h>
#include <tsunami.h>

void setup() {
  Serial.begin(115200);

  // Initialize the Tsunami
  Tsunami.begin();
}

void loop() {
	while(1) {
		Serial.print(Tsunami.measureFrequency());
		Serial.print(" ");
		Serial.println(Tsunami.measurePeakVoltage());
		delay(1000);
	}
}
