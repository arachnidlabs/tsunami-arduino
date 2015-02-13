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
#include <ad983x.h>
#include <mcp4xxx.h>
#include <tsunami.h>

volatile uint16_t last_edge = -1;
volatile uint16_t interval = -1;

/* Called whenever we detect a falling edge on the input signal.
 * This routine measures the interval between edges. It's very time critical;
 * the current version takes about 35 cycles to execute, and so can only measure
 * frequencies up to about 400khz reliably.
 */
ISR(TIMER1_CAPT_vect) {
  interval = ICR1 - last_edge;
  last_edge = ICR1;
}

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  
  // Initialize the Tsunami
  Tsunami.begin();
  
  // Turn on the square wave output so we can measure phase
  Tsunami.setSignOutput(SIGN_OUTPUT_MSB);
  
  // Start with a default frequency of 1khz
  Tsunami.setFrequency(0, 1000.0);
  
  // Initialize timer 1 to run at full speed and capture external events
  TCCR1A &= ~_BV(WGM11) & ~_BV(WGM10);
  TCCR1B &= ~_BV(WGM13) & ~_BV(WGM12) & ~_BV(CS12) & ~_BV(CS11);
  TCCR1B |= _BV(CS10);
  TIMSK1 |= _BV(ICIE1);
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

/* Measures phase offset, returning a figure between 0 and 1. 0 indicates the
 * signals are 180 degrees out of phase, while 1 indicates the signals are
 * exactly in phase. Note that the analog signal chain imposes some delay,
 * meaning that at higher frequencies there is a significant phase shift that
 * must be accounted for to get accurate measurements; this function does not
 * make any adjustment for that.
 */
float measurePhase() {
  return analogRead(TSUNAMI_PHASE) / 1024.0;
}

/* Measures peak to peak amplitude, returning a value in volts. Note that
 * because of the diode drop on the peak detector, voltages below about 0.16v
 * will register as 0.16v. This effect increases with frequency, which is not
 * accounted for by this function. Also note that a decrease in the amplitude
 * will take some time to show up on the output, as charge leaks from the
 * storage cap. For a more accurate instantaneous reading, set the TSUNAMI_PEAK
 * pin to output, bring it low briefly, then return it to input and wait a while
 * for the capacitor to charge.
 */
float measurePeakVoltage() {
  return (analogRead(TSUNAMI_PEAK) / 1024.0) * 10.0 + 0.32;
}

/* Measures frequency, returning a value in Hz.
 * This works from approximately 250Hz to 400kHz. A more sophisticated routine
 * could measure lower frequencies by reducing the sampling clock rate, and
 * higher frequencies by counting the number of events in a fixed interval
 * instead of the interval between events.
 *
 * Return values are accurate, but will suffer some jitter due to the analog
 * nature of the input signal. Measuring the square wave output will give a more
 * precise result than measuring the sine wave output.
 */
float measureFrequency() {
  return 16000000.0 / interval;
}

// Handles 'freq' commands
void commandFreq(char *name, char *args) {
  setFrequency(atof(args));
  Serial.println("ok");
}

// Handles 'amp' commands
void commandAmp(char *name, char *args) {
  float amp = atof(args);
  int wiper;

  if(amp <= 0) {
    wiper = 256;
  } else {
    // Determined from the Rset and Rfb values and AD9838 parameters
    wiper = (int)round(-(16 * (13915 * amp - 84132)) / (15625 * amp));
  }
  
  if(wiper > 256) {
    wiper = 256;
  }

  Tsunami.setAmplitude(wiper);

  Serial.println("ok");
}

// Handles 'off' commands
void commandOff(char *name, char *args) {
  float offset = -atof(args);
  int wiper;
  
  if(offset == 0.0) {
    wiper = 128;
  } else {
    wiper = (int)round((-0.002048 * sqrt(4300998724.0 * offset * offset + 26265625.0) + 128 * offset + 10.496) / offset);
  }

  if(wiper < 0) {
    wiper = 0;
  } else if(wiper > 256) {
    wiper = 256;
  }
  
  Tsunami.setOffset(wiper);
  
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
  double voltage = measurePeakVoltage();
  double phase = measurePhase();
  double frequency = measureFrequency();
  
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