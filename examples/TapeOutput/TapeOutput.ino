/** 
 * TapeOutput.ino
 * Demonstration of encoding data for storage on tape using a modification of
 * the Kansas City standard (http://en.wikipedia.org/wiki/Kansas_City_standard).
 *
 * This example uses the DDS's support for toggling between two frequencies
 * using a hardware pin by pointing a SoftwareSerial port at it and writing
 * data, which then gets translated directly into frequency-shift keyed data,
 * just like 8-bit home computers used to store and retrieve data on tape.
 */

#include <SoftwareSerial.h>
#include <SPI.h>
#include <tsunami.h>

// Create a SoftwareSerial instance that writes to the DDS's frequency
// select pin
SoftwareSerial out(TSUNAMI_DDS_FSEL, TSUNAMI_DDS_FSEL);

void setup() {
  // Initialize the Tsunami
  Tsunami.begin();
  
  // Turn output off
  Tsunami.sleep(true);
  
  // Set the frequencies for 0 and 1 bits
  Tsunami.setFrequency(0, 1200l);
  Tsunami.setFrequency(1, 2400l);
  
  // Initialize the serial output at 300 baud
  out.begin(300);
}

void loop() {
  while(1) {
    // Wait until there's data
    while(!Serial.available());
    
    // Turn the output on and emit a short leader of 1 bits
    Tsunami.sleep(false);
    delay(3);
    
    // While there's data, write it out on the serial interface
    while(Serial.available()) {
      out.write(Serial.read());
    }
    
    // Include a short trailer of 1 bits before disabling the output again
    delay(3);
    Tsunami.sleep(true);
  }
}
