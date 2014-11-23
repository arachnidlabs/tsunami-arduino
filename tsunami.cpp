#include <tsunami.h>

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
}

Tsunami_Class Tsunami;
