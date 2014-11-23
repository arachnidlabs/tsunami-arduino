#ifndef __ARDUIN_TSUNAMI_H
#define __ARDUINO_TSUNAMI_H

#include <ad983x.h>
#include <mcp4xxx.h>

#define TSUNAMI_DDS_CS        8
#define TSUNAMI_DDS_FSEL      5
#define TSUNAMI_DDS_PSEL      6
#define TSUNAMI_DDS_SLEEP     9
#define TSUNAMI_DDS_RESET     10
#define TSUNAMI_DDS_SIGN      A0
#define TSUNAMI_PHASE         A1
#define TSUNAMI_PEAK          A2
#define TSUNAMI_VIN           A3
#define TSUNAMI_FREQIN_1      4
#define TSUNAMI_FREQIN_2      12
#define TSUNAMI_DIGIPOT_CS    7
#define TSUNAMI_OFFSET_POT    MCP4XXX::pot_0
#define TSUNAMI_AMPLITUDE_POT MCP4XXX::pot_1
#define TSUNAMI_FREQUENCY     16 // MHz

class Tsunami_Class {
public:
  Tsunami_Class();
  void begin();

  inline void setFrequencyWord(byte reg, uint32_t frequency) {
    dds.setFrequencyWord(reg, frequency);
  }

  inline void setPhaseWord(byte reg, uint32_t phase) {
    dds.setPhaseWord(reg, phase);
  }

  inline void setSignOutput(SignOutput out) {
    dds.setSignOutput(out);
  }

  inline void setOutputMode(OutputMode out) {
    dds.setOutputMode(out);
  }

  inline void setFrequency(byte reg, long int frequency)  {
    dds.setFrequency(reg, frequency);
  }

  inline void setFrequency(byte reg, float frequency) {
    dds.setFrequency(reg, frequency);
  }

  inline void reset(boolean in_reset) {
    dds.reset(in_reset);
  }

  inline void sleep(boolean sleeping) {
    digitalWrite(TSUNAMI_DDS_SLEEP, sleeping);
  }

  inline void selectFrequency(byte reg) {
    digitalWrite(TSUNAMI_DDS_FSEL, reg);
  }

  inline void selectPhase(byte reg) {
    digitalWrite(TSUNAMI_DDS_PSEL, reg);
  }

  inline void setAmplitude(int amp) {
    amplitude.set(amp);
  }

  inline void setOffset(int off) {
    offset.set(off);
  }

private:
  AD983X_PIN dds;
  MCP4XXX offset;
  MCP4XXX amplitude;
};

extern Tsunami_Class Tsunami;

#endif
