/** 
 * Calibration.ino
 * Interactive calibration of input and output voltages of the Tsunami.
 * Requires a generic digital multimeter for the output calibration phase
 * and an output-to-input loopback connection for input calibration phase.
 *
 * Accepts commands via the serial port at 115200 baud.
 */

#include <SPI.h>
#include <EEPROM.h>
#include <tsunami.h>

#define BUFFER_SIZE     32
#define SAMPLE_SIZE     16
#define NUM_MEASURE     32

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  
  // Initialize the Tsunami
  Tsunami.begin();
}

// Waits until a newline arrives from the serial port
void waitLine() {
  char key;
  while(1) {
    if(Serial.available()) {
      key = Serial.read();

      if(key == '\n')
        return;
    }
  }
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

int32_t roundToInt32(float value) {
  if(value > 0) {
    return (int32_t)(value + 0.5);
  } else {
    return (int32_t)(value - 0.5);
  }
}

// Calculates the linear regression coefficients that satisfy y=a*x+b
// with least amount of total error (sum of error squares is minimal)
// Note: it is assumed that x and y values are all within +/-3300 and
// n is less than 98 otherwise the int32 sums could possibly overflow
void regress(int32_t *x, int32_t *y, int n, float *a, float *b) {
  int32_t avgx = 0;
  int32_t avgy = 0;
  int32_t ssxx = 0;
  int32_t ssxy = 0;

  // These are called averages but they're just added up sums for now
  for(int i = 0; i < n; i++) {
    avgx += x[i];
    avgy += y[i];
  }

  // Now these become actual averages, rounded to the nearest integer
  avgx = roundToInt32((float)avgx / n);
  avgy = roundToInt32((float)avgy / n);

  // These are called "variance of x" and "covariance of x and y"; we
  // should actually divide both sums by n in the end to get the real
  // values but since all we want is their proportion there's no need
  for(int i = 0; i < n; i++) {
    ssxx += (x[i] - avgx) * (x[i] - avgx);
    ssxy += (x[i] - avgx) * (y[i] - avgy);
  }

  // The slope of the regression a is covariance divided by variance;
  // the intercept b is not hard to get as soon as the slope is known
  *a = (float)ssxy / (float)ssxx;
  *b = (float)avgy - *a * (float)avgx;
}

void loop() {
  char buf[BUFFER_SIZE];
  uint8_t value;
  float scale, shift;
  float slope, intercept;
  float cal_scale[CAL_DATA_ALL];
  float cal_shift[CAL_DATA_ALL];
  int32_t target_ofs[SAMPLE_SIZE];
  int32_t target_amp[SAMPLE_SIZE];
  int32_t target_val[SAMPLE_SIZE];
  int32_t actual_ofs[SAMPLE_SIZE];
  int32_t actual_amp[SAMPLE_SIZE];
  int32_t actual_val[SAMPLE_SIZE];
  int32_t actual_zero_offset;
  uint8_t number_ofs;
  uint8_t number_amp;

  // We sure don't want to calibrate on the back of a previous calibration so let's not
  Tsunami.useCalibrationData(CAL_DATA_NONE);

  // Wait until someone actually opens the serial port to avoid losing initial greeting
  while(!Serial);

  Serial.println(F("--==< Welcome to Tsunami Calibration Tool 1.0 >==--"));
  Serial.println();

  // Retrieve (without applying) and list the currently stored calibration coefficients
  Serial.println(F("Your currently stored calibration coefficients are:"));
  Serial.println();

  if(Tsunami.getCalibrationData(CAL_DATA_OFFSET, &scale, &shift)) {
    Serial.print(F("Output offset:    scale = "));
    Serial.print(scale, 6);
    Serial.print(F(", shift = "));
    Serial.print(shift/1000, 3);
    Serial.println(F(" V."));
  } else {
    Serial.println(F("Output offset:    no calibration data found..."));
  }

  if(Tsunami.getCalibrationData(CAL_DATA_AMPLITUDE, &scale, &shift)) {
    Serial.print(F("Output amplitude: scale = "));
    Serial.print(scale, 6);
    Serial.print(F(", shift = "));
    Serial.print(shift/1000, 3);
    Serial.println(F(" V."));
  } else {
    Serial.println(F("Output amplitude: no calibration data found..."));
  }

  if(Tsunami.getCalibrationData(CAL_DATA_MEAN_VALUE, &scale, &shift)) {
    Serial.print(F("Input mean:       scale = "));
    Serial.print(scale, 6);
    Serial.print(F(", shift = "));
    Serial.print(shift/1000, 3);
    Serial.println(F(" V."));
  } else {
    Serial.println(F("Input mean:       no calibration data found..."));
  }

  if(Tsunami.getCalibrationData(CAL_DATA_PEAK_VALUE, &scale, &shift)) {
    Serial.print(F("Input peak:       scale = "));
    Serial.print(scale, 6);
    Serial.print(F(", shift = "));
    Serial.print(shift/1000, 3);
    Serial.println(F(" V."));
  } else {
    Serial.println(F("Input peak:       no calibration data found..."));
  }

  if(Tsunami.getCalibrationData(CAL_DATA_CURRENT_VALUE, &scale, &shift)) {
    Serial.print(F("Input instant:    scale = "));
    Serial.print(scale, 6);
    Serial.print(F(", shift = "));
    Serial.print(shift/1000, 3);
    Serial.println(F(" V."));
  } else {
    Serial.println(F("Input instant:    no calibration data found..."));
  }

  Serial.println();
  Serial.print(F("Reset Coefficients, Edit Coefficients or start Calibration Wizard? (r/e/w) "));

  readLine(buf, BUFFER_SIZE);
  
  Serial.println();
  
  if(tolower(buf[0]) == 'r') {
    Serial.print(F("Do you really wish to reset all calibration coefficients? (y/n) "));
    
    readLine(buf, BUFFER_SIZE);
    
    Serial.println();

    if(tolower(buf[0]) == 'y') {
      Tsunami.setCalibrationData(CAL_DATA_OFFSET, 1.0, 0.0);
      Tsunami.setCalibrationData(CAL_DATA_AMPLITUDE, 1.0, 0.0);
      Tsunami.setCalibrationData(CAL_DATA_MEAN_VALUE, 1.0, 0.0);
      Tsunami.setCalibrationData(CAL_DATA_PEAK_VALUE, 1.0, 0.0);
      Tsunami.setCalibrationData(CAL_DATA_CURRENT_VALUE, 1.0, 0.0);
      Serial.println(F("Calibration coefficients were reset."));
    } else {
      Serial.println(F("Calibration coefficients were NOT reset."));
    }    
    
    Serial.println();
    Serial.println(F("Press Enter if you wish to start again."));
    Serial.println();

    waitLine();

    return;
  }

  if(tolower(buf[0]) == 'e') {
    do {
      do {
        Serial.println(F("List of calibration coefficients:"));
        Serial.println(F("1 : Output offset"));
        Serial.println(F("2 : Output amplitude"));
        Serial.println(F("3 : Input mean value"));
        Serial.println(F("4 : Input peak value"));
        Serial.println(F("5 : Input instant value"));
        Serial.print(F("Please select a coefficient to edit: (1-5) "));
    
        readLine(buf, BUFFER_SIZE);
        
        Serial.println();
  
        value = atoi(buf) - 1;

        if(value >= CAL_DATA_ALL) {
          Serial.println(F("Sorry, that is not a value from this list; please try again."));
          Serial.println();
        }
        
      } while(value >= CAL_DATA_ALL);

      Serial.print(F("Your choice is "));
      Serial.print(value + 1);
      Serial.print(F("; is that correct? (y/n) "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();
    } while(tolower(buf[0]) != 'y');

    do {
      Serial.print(F("Please input the new scale factor: "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();

      scale = atof(buf);

      Serial.print(F("Your choice is "));
      Serial.print(scale, 6);
      Serial.print(F("; is that correct? (y/n) "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();
    } while(tolower(buf[0]) != 'y');

    do {
      Serial.print(F("Please input the new offset (in V): "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();

      shift = atof(buf);

      Serial.print(F("Your choice is "));
      Serial.print(shift, 3);
      Serial.print(F(" V; is that correct? (y/n) "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();
    } while(tolower(buf[0]) != 'y');

    // Shift is actually in the units of the calibrated entity (mV), so correct for that
    Tsunami.setCalibrationData((CalibratedValue)value, scale, shift*1000);

    Serial.print(F("Updated item "));
    Serial.print(value + 1);
    Serial.print(F(" to scale = "));
    Serial.print(scale, 6);
    Serial.print(F(" , shift = "));
    Serial.print(shift, 3);
    Serial.println(F(" V."));

    Serial.println();
    Serial.println(F("Press Enter if you wish to start again."));
    Serial.println();

    waitLine();

    return;
  }

  /*
   * Start of Calibration Wizard
   */

  Serial.println(F("Please connect a digital multimeter to the output of your Tsunami,"));
  Serial.println(F("switched to 'DC voltage' on the lowest range that can measure 3V."));
  Serial.println();
  Serial.println(F("Calibration consists of 5 phases: 2 are manual and 3 are automatic."));
  Serial.println(F("First, you will be prompted for your reading on a series of values;"));
  Serial.println(F("Please press Enter to start phase 1."));

  waitLine();

  /*
   * PHASE 1
   */

  Serial.println();
  Serial.println(F("Starting phase 1..."));
  Serial.println();

  // Hold the DDS chip in reset for DC offset measurements
  Tsunami.reset(true);

  number_ofs = 0;

  actual_zero_offset = 0;

  for(int value = -2500; value <= 2500; value += 500 ) {
    Tsunami.setOffset(value);

    target_ofs[number_ofs] = value;

    do {
      Serial.print(F("Testing offset at "));
      Serial.print((float)value/1000, 3);
      Serial.println(F(" V."));
      Serial.print(F("Please input your reading (in V): "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();

      actual_ofs[number_ofs] = roundToInt32(atof(buf) * 1000);

      Serial.print(F("Your reading is "));
      Serial.print((float)actual_ofs[number_ofs]/1000, 3);
      Serial.print(F(" V; is that correct? (y/n) "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();
    } while(tolower(buf[0]) != 'y');

    // Store the measured offset corresponding to a setting of zero offset for later use
    if(target_ofs[number_ofs] == 0) {
      actual_zero_offset = actual_ofs[number_ofs];
    }

    number_ofs++;
  }

  // Release the DDS chip from reset
  Tsunami.reset(false);

  // Calculate best fitting line based on a list of output offsets we try to set and the
  // output offsets actually produced by the Tsunami as measured and entered by the user
  regress(target_ofs, actual_ofs, number_ofs, &slope, &intercept);

  // Calculate the calibration scale and shift that would cancel out such a linear error
  cal_scale[CAL_DATA_OFFSET] = 1.0 / slope;
  cal_shift[CAL_DATA_OFFSET] = - intercept / slope;

  Serial.println();
  Serial.println(F("Phase 1 complete."));
  Serial.println();
  Serial.println(F("Next, please switch your DMM to 'AC voltage', lowest range for 2.5V."));
  Serial.println();
  Serial.println(F("Again, you will be prompted for your reading on a series of values;"));
  Serial.println(F("Please press Enter to start phase 2."));

  waitLine();

  /*
   * PHASE 2
   */

  Serial.println();
  Serial.println(F("Starting phase 2..."));
  Serial.println();

  // Reset the offset and select a frequency of 1kHz
  Tsunami.setOffset(0);
  Tsunami.setFrequency(0, 1000.0);
  Tsunami.selectFrequency(0);

  number_amp = 0;

  for(int value = 1000; value <= 6000; value += 1000 ) { 
    Tsunami.setAmplitude(value);

    target_amp[number_amp] = value;

    do {
      Serial.print(F("Testing amplitude at "));
      Serial.print((float)value/1000, 3);
      Serial.print(F(" Vpp ("));
      Serial.print((float)value/2000 / sqrt(2), 3);
      Serial.println(F(" Vrms)."));
      Serial.print(F("Please input your reading (in V): "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();

      actual_amp[number_amp] = roundToInt32(atof(buf) * 1000);

      Serial.print(F("Your reading is "));
      Serial.print((float)actual_amp[number_amp]/1000, 3);
      Serial.print(F(" V; is that correct? (y/n) "));

      readLine(buf, BUFFER_SIZE);

      Serial.println();
    } while(tolower(buf[0]) != 'y');

    // Correct for settings being "peak-to-peak" versus readings being "AC RMS" voltage
    actual_amp[number_amp] *= (2 * sqrt(2));

    number_amp++;
  }

  // Calculate best fitting line based on output amplitudes we are trying to set and the
  // output amplitudes actually produced by the Tsunami measured and entered by the user
  regress(target_amp, actual_amp, number_amp, &slope, &intercept);

  // Calculate the calibration scale and shift that would cancel out such a linear error
  cal_scale[CAL_DATA_AMPLITUDE] = 1.0 / slope;
  cal_shift[CAL_DATA_AMPLITUDE] = - intercept / slope;

  Serial.println();
  Serial.println(F("Phase 2 complete."));
  Serial.println();
  Serial.println(F("Next, please detach your DMM and loop Tsunami's output to its input."));
  Serial.println();
  Serial.println(F("The hard part is over - the remaining three phases are automatic;"));
  Serial.println(F("Please press Enter to start phase 3."));

  waitLine();

  /*
   * PHASE 3
   */

  Serial.println();
  Serial.println(F("Starting phase 3..."));
  Serial.println();

  // Set a small signal to avoid distortions at high offsets
  Tsunami.setAmplitude(1000);

  // Note - Yes, we measure DC offset (average) with an AC signal instead of a stable DC
  // offset because the measuring circuit has hysteresis and would return different data
  // for the same fixed DC input depending on the direction of our scan - so AC it is...

  for(int point = 0; point < number_ofs; point++ ) {
    int value = target_ofs[point];

    Tsunami.setOffset(value);

    Serial.print(F("Measuring offset at "));
    Serial.print((float)value/1000, 3);
    Serial.print(F(" V."));

    delay(1000);

    actual_val[point] = 0;

    for (int count = 0; count < NUM_MEASURE; count++) {
      actual_val[point] += Tsunami.measureMeanVoltage();

      delay(100);
    }

    actual_val[point] /= NUM_MEASURE;

    Serial.print(F(" Actual reading: "));
    Serial.print((float)actual_val[point]/1000, 3);
    Serial.println(F(" V."));
  }

  // Calculate best fitting line based on actually produced output offsets as previously
  // measured and reported by the user and "mean input voltage" measurements we just did
  regress(actual_ofs, actual_val, number_ofs, &slope, &intercept);

  // Calculate the calibration scale and shift that would cancel out such a linear error
  cal_scale[CAL_DATA_MEAN_VALUE] = 1.0 / slope;
  cal_shift[CAL_DATA_MEAN_VALUE] = - intercept / slope;

  Serial.println();
  Serial.println(F("Phase 3 complete."));
  Serial.println();
  Serial.println(F("Please leave the Tsunami's output connected to its input."));
  Serial.println();
  Serial.println(F("Please press Enter to start phase 4."));

  waitLine();

  /*
   * PHASE 4
   */

  Serial.println();
  Serial.println(F("Starting phase 4..."));
  Serial.println();

  // Reset the offset and select a frequency of 1kHz
  Tsunami.setOffset(0);
  Tsunami.setFrequency(0, 1000.0);
  Tsunami.selectFrequency(0);

  for(int point = 0; point < number_amp; point++ ) {
    int value = target_amp[point];

    // Correct for the actual, measured output offset at zero offset setting and for the 
    // discrepancy between the "peak-to-peak" setting versus "positive peak" measurement
    target_val[point] = actual_amp[point] / 2 + actual_zero_offset;

    Tsunami.setAmplitude(value);

    Serial.print(F("Measuring amplitude at "));
    Serial.print((float)value/1000, 3);
    Serial.print(F(" Vpp ("));
    Serial.print((float)value/2000, 3);
    Serial.print(F(" Vpeak)."));

    delay(1000);

    actual_val[point] = 0;

    for (int count = 0; count < NUM_MEASURE; count++) {
      actual_val[point] += Tsunami.measurePeakVoltage();

      delay(100);
    }

    actual_val[point] /= NUM_MEASURE;

    Serial.print(" Actual reading: ");
    Serial.print((float)actual_val[point]/1000, 3);
    Serial.println(F(" Vpeak."));
  }

  // Calculate best fitting line based on actually produced output amplitudes previously
  // measured and reported by the user and "peak input voltage" measurements we just did
  regress(target_val, actual_val, number_amp, &slope, &intercept);

  // Calculate the calibration scale and shift that would cancel out such a linear error
  cal_scale[CAL_DATA_PEAK_VALUE] = 1.0 / slope;
  cal_shift[CAL_DATA_PEAK_VALUE] = - intercept / slope;

  Serial.println();
  Serial.println(F("Phase 4 complete."));
  Serial.println();
  Serial.println(F("Please leave the Tsunami's output connected to its input."));
  Serial.println();
  Serial.println(F("Please press Enter to start phase 5."));

  waitLine();

  /*
   * PHASE 5
   */

  Serial.println();
  Serial.println(F("Starting phase 5..."));
  Serial.println();

  // Hey, look! Deja-vu! Hold the DDS chip in reset, again...
  Tsunami.reset(true);

  for(int point = 0; point < number_ofs; point++ ) {
    int value = target_ofs[point];

    Tsunami.setOffset(value);

    Serial.print(F("Measuring instant value at "));
    Serial.print((float)value/1000, 3);
    Serial.print(F(" V."));

    delay(1000);

    actual_val[point] = 0;

    for (int count = 0; count < NUM_MEASURE; count++) {
      actual_val[point] += Tsunami.measureCurrentVoltage();

      delay(100);
    }

    actual_val[point] /= NUM_MEASURE;
    
    Serial.print(F(" Actual reading: "));
    Serial.print((float)actual_val[point]/1000, 3);
    Serial.println(F(" V."));
  }

  // Release the DDS chip from reset (again...)
  Tsunami.reset(false);

  // Calculate best fitting line based on actually produced output offsets as previously
  // measured and reported by the user and "current input voltage" data we just measured
  regress(actual_ofs, actual_val, number_ofs, &slope, &intercept);

  // Calculate the calibration scale and shift that would cancel out such a linear error
  cal_scale[CAL_DATA_CURRENT_VALUE] = 1.0 / slope;
  cal_shift[CAL_DATA_CURRENT_VALUE] = - intercept / slope;

  Serial.println();
  Serial.println(F("Phase 5 complete."));
  Serial.println();
  Serial.println(F("Calibration complete. The recommended calibration coefficients are:"));
  Serial.println();
  Serial.print(F("Output offset: scale = "));
  Serial.print(cal_scale[CAL_DATA_OFFSET], 6);
  Serial.print(F(", shift = "));
  Serial.print(cal_shift[CAL_DATA_OFFSET]/1000, 3);
  Serial.println(F(" V."));
  Serial.print(F("Output amplitude: scale = "));
  Serial.print(cal_scale[CAL_DATA_AMPLITUDE], 6);
  Serial.print(F(", shift = "));
  Serial.print(cal_shift[CAL_DATA_AMPLITUDE]/1000, 3);
  Serial.println(F(" V."));
  Serial.print(F("Input mean: scale = "));
  Serial.print(cal_scale[CAL_DATA_MEAN_VALUE], 6);
  Serial.print(F(", shift = "));
  Serial.print(cal_shift[CAL_DATA_MEAN_VALUE]/1000, 3);
  Serial.println(F(" V."));
  Serial.print(F("Input peak: scale = "));
  Serial.print(cal_scale[CAL_DATA_PEAK_VALUE], 6);
  Serial.print(F(", shift = "));
  Serial.print(cal_shift[CAL_DATA_PEAK_VALUE]/1000, 3);
  Serial.println(F(" V."));
  Serial.print(F("Input instant: scale = "));
  Serial.print(cal_scale[CAL_DATA_CURRENT_VALUE], 6);
  Serial.print(F(", shift = "));
  Serial.print(cal_shift[CAL_DATA_CURRENT_VALUE]/1000, 3);
  Serial.println(F(" V."));
  Serial.println();
  Serial.print(F("Would you like to apply and store these calibration coefficients? (y/n) "));

  readLine(buf, BUFFER_SIZE);

  Serial.println();

  if(tolower(buf[0]) == 'y') {
    Tsunami.setCalibrationData(CAL_DATA_OFFSET, cal_scale[CAL_DATA_OFFSET], cal_shift[CAL_DATA_OFFSET]);
    Tsunami.setCalibrationData(CAL_DATA_AMPLITUDE, cal_scale[CAL_DATA_AMPLITUDE], cal_shift[CAL_DATA_AMPLITUDE]);
    Tsunami.setCalibrationData(CAL_DATA_MEAN_VALUE, cal_scale[CAL_DATA_MEAN_VALUE], cal_shift[CAL_DATA_MEAN_VALUE]);
    Tsunami.setCalibrationData(CAL_DATA_PEAK_VALUE, cal_scale[CAL_DATA_PEAK_VALUE], cal_shift[CAL_DATA_PEAK_VALUE]);
    Tsunami.setCalibrationData(CAL_DATA_CURRENT_VALUE, cal_scale[CAL_DATA_CURRENT_VALUE], cal_shift[CAL_DATA_CURRENT_VALUE]);
    Serial.println(F("Calibration coefficients were applied and stored."));
  } else {
    Serial.println(F("Calibration coefficients were NOT applied / stored."));
  }

  Serial.println();
  Serial.println(F("Press Enter if you wish to start again."));
  Serial.println();

  waitLine();
}
