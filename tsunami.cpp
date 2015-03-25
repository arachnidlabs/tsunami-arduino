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

Tsunami_Class::Tsunami_Class() :
	dds(TSUNAMI_DDS_CS, TSUNAMI_FREQUENCY, TSUNAMI_DDS_RESET),
	offset(TSUNAMI_DIGIPOT_CS, TSUNAMI_OFFSET_POT),
	amplitude(TSUNAMI_DIGIPOT_CS, TSUNAMI_AMPLITUDE_POT)
{
}

void Tsunami_Class::begin() {
	pinMode(TSUNAMI_DDS_SLEEP, OUTPUT);
	digitalWrite(TSUNAMI_DDS_SLEEP, LOW);

	pinMode(TSUNAMI_DDS_FSEL, OUTPUT);
	digitalWrite(TSUNAMI_DDS_FSEL, LOW);

	pinMode(TSUNAMI_DDS_PSEL, OUTPUT);
	digitalWrite(TSUNAMI_DDS_PSEL, LOW);

	amplitude.set(0);
	offset.set(128);

	current_reg = 0;

	// Initialize timer 1 to run at full speed and capture external events
	//TCCR1A &= ~_BV(WGM11) & ~_BV(WGM10);
	//TCCR1B &= ~_BV(WGM13) & ~_BV(WGM12) & ~_BV(CS12) & ~_BV(CS11);
	//TCCR1B |= _BV(CS10);
	//TIMSK1 |= _BV(ICIE1);

	dds.begin();
}

float Tsunami_Class::measureFrequency() {
    // TODO: Enhance to support a wider range of frequencies.
    return 16000000.0 / interval;
}

Tsunami_Class Tsunami;
