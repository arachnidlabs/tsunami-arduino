/** 
 * FrequencyGenerator.ino
 * Demonstration of frequency generation and measurement using the Tsunami.
 *
 * Accepts commands via the serial port at 115200 baud.
 *
 * Valid commands:
 *   freq x
 *     Sets the frequency to the specified value in hz.
 *   amp x
 *     Sets the peak-to-peak amplitude to the specified voltage between roughly
 *     0.32 and 6.15Vp-p. Note that the accuracy increases significantly for
 *     smaller voltages, so large voltages may be approximate.
 *   off x
 *     Sets the DC offset to the specified voltage between roughly +2 and -2
 *     volts. Note that the accuracy around 0 is significantly better than for
 *     larger offsets.
 *   wf (sine|triangle)
 *     Sets the waveform output to sine or triangle.
 *   measure
 *     Measures the waveform on the input connector, returning "measured freq phase amp"
 *     where freq is the frequency in hz, phase is the floating point phase offset in degrees, and amp is the
 *     peak-peak amplitude in volts.
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

// Sets the output frequency to the specified value
void setFrequency(float freq) {
  // We alternate which register we use to avoid glitches in the output.
  static uint8_t frequency_register = 1;

  Tsunami.setFrequency(frequency_register, freq);
  Tsunami.selectFrequency(frequency_register);
  frequency_register = 1 - frequency_register;
}

// Reads a line from the serial port, without timeout
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

// Handles 'freq' commands
void commandFreq(char *name, char *args) {
  setFrequency(atof(args));
  Serial.println("ok");
}

// Handles 'amp' commands
void commandAmp(char *name, char *args) {
  int value = (int)(atof(args) * 1000);
  Tsunami.setAmplitude(value);

  Serial.println("ok");
}

// Handles 'off' commands
void commandOff(char *name, char *args) {
  int value = (int)(atof(args) * 1000);
  Tsunami.setOffset(value);
  
  Serial.println("ok");
}

// Handles 'wf' commands
void commandWf(char *name, char *args) {
  if(args[0] == 't' || args[0] == 'T') {
    Tsunami.setOutputMode(OUTPUT_MODE_TRIANGLE);
  } else {
    Tsunami.setOutputMode(OUTPUT_MODE_SINE);
  }
  
  Serial.println("ok");
}

void commandMeasure(char *name, char *args) {
  double voltage = Tsunami.measurePeakVoltage();
  double phase = Tsunami.measurePhase();
  double frequency = Tsunami.measureFrequency();
  
  Serial.print("measured ");
  Serial.print(frequency);
  Serial.print(" ");
  Serial.print(phase);
  Serial.print(" ");
  Serial.println(voltage);
}

// This is a shorthand name for a pointer to a function that handles a command
typedef void (*command_handler)(char *, char *);

// A command_t structure holds the name of a command and the function that
// should handle it
typedef struct {
  char *name;
  command_handler handler;
} command_t;

// An array of all the valid command handlers. An empty entry denotes the
// end of the list.
const command_t commands[] = {
  {"freq", &commandFreq},
  {"amp", &commandAmp},
  {"off", &commandOff},
  {"wf", &commandWf},
  {"measure", &commandMeasure},
  {NULL, NULL}
};

// Interprets a command and executes the relevant function
void handleCommand(char *buf) {
  // Separate the command from its arguments
  char *name = strsep(&buf, " \n\r");
  
  // Iterate over all the defined commands
  for(const command_t *command = commands; command->name != NULL; command++) {
    // Check if the current command matches
    if(strcasecmp(command->name, name) == 0) {
      // Execute the command handler
      command->handler(name, buf);
      return;
    }
  }
  
  // If we reach this point, the command requested doesn't match anything.
  Serial.print("err Unknown command ");
  Serial.println(name);
}

void loop() {
  char buf[40];
  
  while(1) {
    readLine(buf, 40);
    handleCommand(buf);
  }
}