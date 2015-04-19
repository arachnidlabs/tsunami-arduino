/** 
 * SineOutput.ino
 * Generates a 400Hz sine wave using the Tsunami.
 */

#include <SPI.h>
#include <tsunami.h>

void setup() {
  // Initialize the Tsunami
  Tsunami.begin();
}

void loop() {
	float frequency;
	while(1) {
		frequency = 110.0;
		while(frequency < 8000.0) {
			Tsunami.setFrequency(frequency);
			frequency *= 1.05946309; // 12th root of 2
			delay(500);
		}
	}
}
