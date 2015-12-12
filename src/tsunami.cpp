#include <tsunami.h>

typedef enum {
  COUNTER_STATUS_INVALID,
  COUNTER_STATUS_PENDING,
  COUNTER_STATUS_VALID
} counter_status_t;

static volatile uint16_t last_edge = -1;
static volatile counter_status_t counter_status = COUNTER_STATUS_INVALID;
static volatile uint8_t counter_divider = 0;
static volatile uint16_t counter_topword = 0;
static volatile uint32_t instant_interval = 0;
static volatile uint32_t average_interval = 0;

calibration_data_t cal_data[CAL_DATA_ALL];

/* This implements a counter state machine:
 * Initial state: COUNTER_STATUS_INVALID
 * COUNTER_STATUS_INVALID:  Falling edge  ->  COUNTER_STATUS_PENDING
 * COUNTER_STATUS_PENDING:  Falling edge  ->  if value low AND divider++
 *                                            then COUNTER_STATUS_INVALID,
 *                                            else COUNTER_STATUS_VALID
 * COUNTER_STATUS_VALID:    Falling edge  ->  if value low AND divider++
 *                                            then COUNTER_STATUS_INVALID,
 *                                            else COUNTER_STATUS_VALID
 * ANY:                          Overflow ->  if value high AND divider--
 *                                            then COUNTER_STATUS_INVALID
 */

static inline void _set_divider() {
  digitalWrite(TSUNAMI_FDIV_SEL_0, counter_divider & (1 << 2));
  digitalWrite(TSUNAMI_FDIV_SEL_1, counter_divider & (1 << 3));
}

static inline uint8_t _increase_divider() {
  if(counter_divider < 12) {
    counter_divider += 4;
    _set_divider();
    return 1;
  }
  return 0;
}

static inline uint8_t _reduce_divider() {
  if(counter_divider >= 4) {
    counter_divider -= 4;
    _set_divider();
    return 1;
  }
  return 0;
}

ISR(TIMER1_CAPT_vect) {
  uint16_t this_edge;
  
  // It's good practice to only read a "live" register once: store ICR1
  this_edge = ICR1;
  
  // If there isn't any recent edge capture count recorded ("invalid"),
  // just move into "pending" state and wait for a second falling edge;
  // Otherwise calculate the full count (including any roll-overs) from
  // the last edge to this one, then multiply it by sixteen; we do that
  // because even though the next step - calculating a moving average - 
  // is quite useful to smooth out things, it has the drawback of never
  // actually reaching the exact measured value it filters; however, by
  // feeding it sixteen times the real value to be filtered the average
  // gets a chance to converge close enough to its target that dividing
  // it back by sixteen at the end returns the exact value that we seek
  switch(counter_status) {
  case COUNTER_STATUS_INVALID:
    average_interval = 0;
    counter_status = COUNTER_STATUS_PENDING;
    break;    
  default:  
    instant_interval = (((uint32_t)(counter_topword) << 16) + this_edge - last_edge) << 4;
    
    // Same thing as Iavg = 1/8*Inew + 7/8*Iavg but it gets rid of "x7"
    if (average_interval == 0) {
      average_interval = instant_interval;
    } else {
      average_interval += (int32_t)(instant_interval - average_interval) >> 3;
    }
    
    // 0x50000 is the interval count for 3.2MHz, our upper limit point;
    // 0x100000 (1 << 16 times sixteen) is our low-count limit where we
    // switch dividers; it's about one divider step (x16) times smaller
    // than our other switching threshold, assuring that the count will
    // still fit below that threshold after the divider gets increased;
    // The ratio of the two thresholds is not exactly 1:16 though since
    // we want some hysteresis there to avoid endless up/down switching
    if(((instant_interval < 0x100000) && (_increase_divider())) || (instant_interval < 0x50000)) {
      counter_status = COUNTER_STATUS_INVALID;
    } else {
      counter_status = COUNTER_STATUS_VALID;
    }
    break;
  }
  
  // Store the current edge time until we capture the next falling edge
  last_edge = this_edge;
  
  // Start counting the number of times Timer1 rolls over between edges
  counter_topword = 0;
}

ISR(TIMER1_OVF_vect) {
  // 272 << 16 is the approximate count for 0.9Hz, our lower limit point;
  // 16 << 16 is our high-count threshold for divider switching; we could
  // keep counting actually, but the divided signal would drop under 15Hz
  // and as a direct consequence the responsiveness would start to suffer
  if(((counter_topword > 16) && (_reduce_divider())) || (counter_topword > 272)) {
    counter_topword = 0;
    counter_status = COUNTER_STATUS_INVALID;
  } else {
    counter_topword++;
  }
}

typedef struct {
  uint8_t cs_pin;
  uint8_t bitorder;
  uint8_t mode;
} spi_settings_t;

spi_settings_t dds_spi_settings = {TSUNAMI_DDS_CS, MSBFIRST, SPI_MODE2};
spi_settings_t dac_spi_settings = {TSUNAMI_DAC_CS, MSBFIRST, SPI_MODE0};

// Generic SPI transfer function
static void spi_transfer(void *ctx, char data[], int len) {
  spi_settings_t *settings = (spi_settings_t*)ctx;

  SPI.setBitOrder(settings->bitorder);
  SPI.setDataMode(settings->mode);

  digitalWrite(settings->cs_pin, LOW);
  delayMicroseconds(1);
  for(int i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  delayMicroseconds(1);
  digitalWrite(settings->cs_pin, HIGH);
}

Tsunami_Class::Tsunami_Class() {
  for(int i = 0; i < CAL_DATA_ALL; i++) {
    cal_data[i].scale = 1.0;
    cal_data[i].shift = 0.0;
  }
  ad983x_init(&dds, spi_transfer, &dds_spi_settings);
}

void Tsunami_Class::begin() {
  // DDS control pin config.
  pinMode(TSUNAMI_DDS_SLEEP, OUTPUT);
  digitalWrite(TSUNAMI_DDS_SLEEP, LOW);

  pinMode(TSUNAMI_DDS_FSEL, OUTPUT);
  digitalWrite(TSUNAMI_DDS_FSEL, LOW);

  pinMode(TSUNAMI_DDS_PSEL, OUTPUT);
  digitalWrite(TSUNAMI_DDS_PSEL, LOW);

  pinMode(TSUNAMI_DAC_CS, OUTPUT);
  digitalWrite(TSUNAMI_DAC_CS, HIGH);

  pinMode(TSUNAMI_DDS_CS, OUTPUT);
  digitalWrite(TSUNAMI_DDS_CS, HIGH);

  // Other control pins
  pinMode(TSUNAMI_SIGN_EN, OUTPUT);
  digitalWrite(TSUNAMI_SIGN_EN, LOW);

  // Frequency divider selection pin config.
  pinMode(TSUNAMI_FDIV_SEL_0, OUTPUT);
  pinMode(TSUNAMI_FDIV_SEL_1, OUTPUT);
  _set_divider();

  // Enable SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // Initialize DDS
  pinMode(TSUNAMI_DDS_RESET, OUTPUT);
  digitalWrite(TSUNAMI_DDS_RESET, HIGH);
  ad983x_start(&dds);
  ad983x_set_sign_output(&dds, AD983X_SIGN_OUTPUT_MSB);
  digitalWrite(TSUNAMI_DDS_RESET, LOW);

  // Initialize DAC
  mcp49xx_init(&dac, spi_transfer, &dac_spi_settings);

  // Configure amplitude DAC
  mcp49xx_set_is_buffered(&dac, TSUNAMI_AMPLITUDE_ID, true);
  mcp49xx_set_gain(&dac, TSUNAMI_AMPLITUDE_ID, MCP49XX_GAIN_1X);
  mcp49xx_set_is_shutdown(&dac, TSUNAMI_AMPLITUDE_ID, false);
  mcp49xx_write(&dac, TSUNAMI_AMPLITUDE_ID, 0);

  // Configure offset DAC
  mcp49xx_set_is_buffered(&dac, TSUNAMI_OFFSET_ID, true);
  mcp49xx_set_gain(&dac, TSUNAMI_OFFSET_ID, MCP49XX_GAIN_2X);
  mcp49xx_set_is_shutdown(&dac, TSUNAMI_OFFSET_ID, false);
  mcp49xx_write(&dac, TSUNAMI_OFFSET_ID, 2048);

  // Configure ADC to use internal 2.56V reference
  setAnalogRef(INTERNAL);

  current_frequency_reg = 0;
  current_phase_reg = 0;

  // Initialize timer 1 to run at 16MHz and capture external events
  TCCR1A &= ~_BV(WGM11) & ~_BV(WGM10);
  TCCR1B &= ~_BV(WGM13) & ~_BV(WGM12) & ~_BV(CS12) & ~_BV(CS11);
  TCCR1B |= _BV(ICNC1) | _BV(CS10);
  TIMSK1 |= _BV(ICIE1) | _BV(TOIE1);

  // Enable PWM output on pin 10 from timer 4, so analogWrite works
  TCCR4A |= _BV(PWM4B);

  // Uncomment to increase pin 10 PWM frequency from ~500hz to ~8KHz
  // TCCR4B &= ~_BV(CS42);
}

/* Applies and saves calibration data for a single value; "scale" does not
 * have a unit and "shift" is in whatever unit the value has (millivolts).
 */
uint8_t Tsunami_Class::setCalibrationData(CalibratedValue value, float scale, float shift) {
  calibration_record_t cal_record;
  uint8_t result = 0;

  // This can only access a specific value; no global requests allowed
  if(value < CAL_DATA_ALL) {
    cal_record.magic = TSUNAMI_CALIBRATION;
    cal_record.scale = scale;
    cal_record.shift = shift;

    // Write then read back to check whether the data was really saved
    EEPROM.put(sizeof(cal_record) * value, cal_record);
    EEPROM.get(sizeof(cal_record) * value, cal_record);

    // Verify the retrieved data and if it is saved correctly apply it
    if((cal_record.magic == TSUNAMI_CALIBRATION) &&
      (cal_record.scale == scale) && (cal_record.shift == shift)) {
      cal_data[value].scale = scale;
      cal_data[value].shift = shift;
      result = 1;
    }
  }

  return result;
}

/* Returns the saved calibration data for a single value; "scale" does not
 * have a unit and "shift" is in whatever unit the value has (millivolts);
 * Note: this is the stored calibration data, not the current data in use!
 */
uint8_t Tsunami_Class::getCalibrationData(CalibratedValue value, float *scale, float *shift) {
  calibration_record_t cal_record;
  uint8_t result = 0;

  // This can only access a specific value; no global requests allowed
  if(value < CAL_DATA_ALL) {
    EEPROM.get(sizeof(cal_record) * value, cal_record);

    // Verify the retrieved signature and if it passes return the data
    if(cal_record.magic == TSUNAMI_CALIBRATION) {
      *scale = cal_record.scale;
      *shift = cal_record.shift;
      result = 1;
    }
  }

  return result;
}

/* Applies saved calibration data for either a specific value, all values,
 * or none of them: restores scale 1.0 and shift 0.0 but keeps saved data.
 */
uint8_t Tsunami_Class::useCalibrationData(CalibratedValue value) {
  calibration_record_t cal_record;
  uint8_t this_value, last_value;
  uint8_t result = 1;

  // The value can be either a specific one or global ("all" / "none")
  if(value < CAL_DATA_ALL) {
    this_value = value;
    last_value = value;
  } else {
    this_value = 0;
    last_value = CAL_DATA_ALL;
  }

  // A global value will loop for all values, a specific one only once
  do {
    if(value > CAL_DATA_ALL) {
      cal_data[this_value].scale = 1.0;
      cal_data[this_value].shift = 0.0;
    } else {
      EEPROM.get(sizeof(cal_record) * this_value, cal_record);

      if(cal_record.magic == TSUNAMI_CALIBRATION) {
        cal_data[this_value].scale = cal_record.scale;
        cal_data[this_value].shift = cal_record.shift;
      } else {
        result = 0;
      }
    }
    this_value++;
  } while (this_value < last_value);

  return result;
}

/* Returns the measured frequency on the Tsunami input, in hertz.
 * If no valid signal is measured, or the Tsunami is not done measuring, NAN
 * is returned.
 */
float Tsunami_Class::measureFrequency() {
    uint32_t signal_period;
    uint8_t signal_divider;
    float ret;

    // Disable interrupts so we get a consistent reading from the registers
    cli();

    // Retrieve the instant interval length (multiplied by sixteen) and the
    // divider expressed as the power of two by which the input was divided
    signal_period = instant_interval;
    signal_divider = counter_divider;

    // Re-enable interrupts while the time consuming float division is done
    sei();
        
    // Calculate the frequency if available, not forgetting that the signal
    // period needs to be divided by sixteen to get the real counted value;
    // Adding "8" to the 16x signal period is just the equivalent of adding
    // "0.5" to it so it gets rounded to the nearest value, not simply down
    if(counter_status == COUNTER_STATUS_VALID) {
      ret = (16000000.0 / ((signal_period + 8) >> 4)) * (1 << signal_divider);
    } else {
      ret = NAN;
    }

    return ret;
}

/* Same as above, but with a moving average ratio of "1/8 new data" applied.
 * Ordinarily it tracks to the instant value rather quickly but whenever the
 * divided signal is a really low frequency the reaction time is observable.
 */
float Tsunami_Class::measureAverageFrequency() {
    uint32_t signal_period;
    uint8_t signal_divider;
    float ret;

    // Disable interrupts so we get a consistent reading from the registers
    cli();

    // Retrieve the average interval length (multiplied by sixteen) and the
    // divider expressed as the power of two by which the input was divided
    signal_period = average_interval;
    signal_divider = counter_divider;

    // Re-enable interrupts while the time consuming float division is done
    sei();

    // Calculate the frequency if available, not forgetting that the signal
    // period needs to be divided by sixteen to get the real counted value;
    // Adding "8" to the 16x signal period is just the equivalent of adding
    // "0.5" to it so it gets rounded to the nearest value, not simply down
    if(counter_status == COUNTER_STATUS_VALID) {
      ret = (16000000.0 / ((signal_period + 8) >> 4)) * (1 << signal_divider);
    } else {
      ret = NAN;
    }

    return ret;
}

Tsunami_Class Tsunami;
