#include <tsunami.h>

typedef enum {
  COUNTER_STATUS_INVALID,
  COUNTER_STATUS_VALID,
  COUNTER_STATUS_VALID_OVF,
  COUNTER_STATUS_PENDING
} counter_status_t;

static volatile uint16_t last_edge = -1;
static volatile uint16_t interval = -1;
static volatile counter_status_t counter_status = COUNTER_STATUS_PENDING;
static volatile uint8_t counter_divider = 0;


/* This implements a counter state machine:
 * Initial state: COUNTER_STATUS_PENDING
 * COUNTER_STATUS_PENDING:  Falling edge  ->  COUNTER_STATUS_VALID
 *                          Overflow      ->  COUNTER_STATUS_INVALID, divider--
 * COUNTER_STATUS_INVALID:  Falling edge  ->  COUNTER_STATUS_PENDING
 *                          Overflow      ->  COUNTER_STATUS_INVALID, divider--
 * COUNTER_STATUS_VALID:    Falling edge  ->  COUNTER_STATUS_VALID
 *                          Overflow      ->  COUNTER_STATUS_VALID_OVF
 * COUNTER_STATUS_VALID_OVF:Falling edge  ->  COUNTER_STATUS_VALID
 *                          Overflow      ->  COUNTER_STATUS_INVALID, divider--
 * ANY                      Low count     ->  COUNTER_STATUS_INVALID, divider++
 */

static inline void _set_divider() {
  digitalWrite(TSUNAMI_FDIV_SEL_0, counter_divider & (1 << 2));
  digitalWrite(TSUNAMI_FDIV_SEL_1, counter_divider & (1 << 3));
}

static inline void _increase_divider() {
  if(counter_divider < 12) {
    counter_divider += 4;
    _set_divider();
    counter_status = COUNTER_STATUS_INVALID;
  }
}

static inline void _reduce_divider() {
  if(counter_divider >= 4) {
    counter_divider -= 4;
    _set_divider();
    counter_status = COUNTER_STATUS_INVALID;
  }
}

ISR(TIMER1_CAPT_vect) {
  interval = ICR1 - last_edge;
  last_edge = ICR1;

  if(interval < 1024) {
    _increase_divider();
  } else {
    switch(counter_status) {
    case COUNTER_STATUS_VALID:
    case COUNTER_STATUS_VALID_OVF:
    case COUNTER_STATUS_PENDING:
      counter_status = COUNTER_STATUS_VALID;
      break;
    case COUNTER_STATUS_INVALID:
      counter_status = COUNTER_STATUS_PENDING;
      break;
    }
  }
}

ISR(TIMER1_OVF_vect) {
  switch(counter_status) {
  case COUNTER_STATUS_VALID:
    counter_status = COUNTER_STATUS_VALID_OVF;
    break;
  default:
    _reduce_divider();
    break;
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
  analogReference(INTERNAL);

	current_frequency_reg = 0;
  current_phase_reg = 0;

	// Initialize timer 1 to run at 2MHz and capture external events
	TCCR1A &= ~_BV(WGM11) & ~_BV(WGM10);
	TCCR1B &= ~_BV(WGM13) & ~_BV(WGM12) & ~_BV(CS12) & ~_BV(CS10);
	TCCR1B |= _BV(CS11);
	TIMSK1 |= _BV(ICIE1) | _BV(TOIE1);

  // Enable PWM output on pin 10 from timer 4, so analogWrite works
  TCCR4A |= _BV(PWM4B);

  // Uncomment to increase pin 10 PWM frequency from ~500hz to ~8KHz
  // TCCR4B &= ~_BV(CS42);
}

/* Returns the measured frequency on the Tsunami input, in hertz.
 * If no valid signal is measured, or the Tsunami is not done measuring, NAN
 * is returned.
 */
float Tsunami_Class::measureFrequency() {
    float ret;

    // Disable interrupts so we get a consistent reading from the registers
    cli();

    // Retrieve and calculate the frequency if available
    if(counter_status == COUNTER_STATUS_VALID || counter_status == COUNTER_STATUS_VALID_OVF) {
      ret = (2000000.0 / interval) * (1 << counter_divider);
    } else {
      ret = NAN;
    }

    // Re-enable interrupts
    sei();

    return ret;
}

Tsunami_Class Tsunami;
