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

extern volatile uint16_t tsunami_last_edge;
extern volatile uint16_t tsunami_interval;

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

  // These overloads automatically set the unused frequency register, then
  // switch to it.
  inline void setFrequency(long int frequency) {
    current_reg = 1 - current_reg;
    dds.setFrequency(current_reg, frequency);
    selectFrequency(current_reg);
  }

  inline void setFrequency(double frequency) {
    current_reg = 1 - current_reg;
    dds.setFrequency(current_reg, (float)frequency);
    selectFrequency(current_reg);
  }

  inline void setFrequency(float frequency) {
    current_reg = 1 - current_reg;
    dds.setFrequency(current_reg, frequency);
    selectFrequency(current_reg);
  }

  inline void setFrequency(byte reg, long int frequency)  {
    dds.setFrequency(reg, frequency);
  }

  inline void setFrequency(byte reg, double frequency) {
    dds.setFrequency(reg, (float)frequency);
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
    //TODO: Accept voltage instead of pot setting values
    amplitude.set(amp);
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
  inline float measurePeakVoltage() {
    // TODO: Provide for calibration of this value
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
  float measureFrequency();

  /* Measures phase offset, returning a figure between 0 and 1. 0 indicates the
   * signals are 180 degrees out of phase, while 1 indicates the signals are
   * exactly in phase. Note that the analog signal chain imposes some delay,
   * meaning that at higher frequencies there is a significant phase shift that
   * must be accounted for to get accurate measurements; this function does not
   * make any adjustment for that.
   */
  inline float measurePhase() {
    // TODO: Provide for frequency calibration
    return analogRead(TSUNAMI_PHASE) / 1024.0;
  }

  inline void setOffset(int off) {
    // TODO: Provide for voltage instead of pot settings for this value
    offset.set(off);
  }
  
private:
  AD983X_PIN dds;
  MCP4XXX offset;
  MCP4XXX amplitude;
  uint8_t current_reg;
};

extern Tsunami_Class Tsunami;

#endif
