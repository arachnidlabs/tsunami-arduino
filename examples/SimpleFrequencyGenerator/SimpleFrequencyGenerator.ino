/** 
 * SimpleFrequencyGenerator.ino
 * Demonstration of simple frequency generation using the Tsunami.
 *
 * Accepts frequencies as floating point numbers on the serial console at
 * 115200 baud, and sets the output frequency accordingly. Amplitude is
 * fixed at maximum and DC offset is fixed at 0.
 */

#include <SPI.h>
#include <tsunami.h>

void setup() {
  // Initialize the serial port
  Serial.begin(115200);

  // Initialize the Tsunami
  Tsunami.begin();
  
  // Start with a default frequency of 1khz
  Tsunami.setFrequency(0, 1000.0);
}

int readLine(char *buf, int len) {
  int pos = 0;
  while(1) {
    if(Serial.available()) {
      buf[pos++] = Serial.read();
  
      if(pos == len || buf[pos - 1] == '\n')
        return pos;
    }
  }
}

void setFrequency(float freq) {
  // We alternate which register we use to avoid glitches in the output.
  static uint8_t frequency_register = 1;

  Tsunami.setFrequency(frequency_register, freq);
  Tsunami.selectFrequency(frequency_register);
  frequency_register = 1 - frequency_register;
}

void loop() {
  char buf[20];
  
  while(1) {
    readLine(buf, 20);
    setFrequency(atof(buf));
  }
}