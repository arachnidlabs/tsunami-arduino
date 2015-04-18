#include <tsunami.h>

static volatile uint16_t last_edge = -1;
static volatile uint16_t interval = -1;

/* Called whenever we detect a falling edge on the input signal.
 * This routine measures the interval between edges. It's very time critical;
 * the current version takes about 35 cycles to execute, and so can only measure
 * frequencies up to about 400khz reliably.
 */
ISR(TIMER1_CAPT_vect) {
  //TODO: Optimise
  interval = ICR1 - last_edge;
  last_edge = ICR1;
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
  mcp49xx_init(&dac, spi_transfer, &dac_spi_settings);
}

void Tsunami_Class::begin() {
	pinMode(TSUNAMI_DDS_SLEEP, OUTPUT);
	digitalWrite(TSUNAMI_DDS_SLEEP, LOW);

	pinMode(TSUNAMI_DDS_FSEL, OUTPUT);
	digitalWrite(TSUNAMI_DDS_FSEL, LOW);

	pinMode(TSUNAMI_DDS_PSEL, OUTPUT);
	digitalWrite(TSUNAMI_DDS_PSEL, LOW);

  pinMode(TSUNAMI_DDS_RESET, OUTPUT);
  digitalWrite(TSUNAMI_DDS_RESET, HIGH);
  ad983x_start(&dds);
  digitalWrite(TSUNAMI_DDS_RESET, LOW);

  mcp49xx_write(&dac, TSUNAMI_OFFSET_ID, 128);
  mcp49xx_write(&dac, TSUNAMI_AMPLITUDE_ID, 0);

	current_reg = 0;

	// Initialize timer 1 to run at full speed and capture external events
	TCCR1A &= ~_BV(WGM11) & ~_BV(WGM10);
	TCCR1B &= ~_BV(WGM13) & ~_BV(WGM12) & ~_BV(CS12) & ~_BV(CS11);
	TCCR1B |= _BV(CS10);
	TIMSK1 |= _BV(ICIE1);
}

float Tsunami_Class::measureFrequency() {
    // TODO: Enhance to support a wider range of frequencies.
    return 16000000.0 / interval;
}

Tsunami_Class Tsunami;
