#ifndef __ARDUIN_TSUNAMI_H
#define __ARDUINO_TSUNAMI_H

#include <Arduino.h>
#include <SPI.h>
extern "C" {
  #include <ad983x/ad983x.h>
  #include <mcp49xx/mcp49xx.h>
}

#define TSUNAMI_DDS_CS        8
#define TSUNAMI_DDS_FSEL      5
#define TSUNAMI_DDS_PSEL      6
#define TSUNAMI_DDS_SLEEP     9
#define TSUNAMI_DDS_RESET     11
#define TSUNAMI_AUX           10
#define TSUNAMI_PHASE         A1
#define TSUNAMI_PEAK          A5
#define TSUNAMI_VIN           A4
#define TSUNAMI_AUX_FILTER    A0
#define TSUNAMI_SIGN_EN       A2
#define TSUNAMI_VAVG          A3
#define TSUNAMI_FREQIN_1      4
#define TSUNAMI_FREQIN_2      12
#define TSUNAMI_DAC_CS        7
#define TSUNAMI_FDIV_SEL_0    31
#define TSUNAMI_FDIV_SEL_1    30
#define TSUNAMI_OFFSET_ID    1
#define TSUNAMI_AMPLITUDE_ID 0
#define TSUNAMI_FREQUENCY     16 // MHz

enum SignOutput {
  SIGN_OUTPUT_NONE        = AD983X_SIGN_OUTPUT_NONE,
  SIGN_OUTPUT_MSB         = AD983X_SIGN_OUTPUT_MSB,
  SIGN_OUTPUT_MSB_2       = AD983X_SIGN_OUTPUT_MSB_2,
  SIGN_OUTPUT_COMPARATOR  = AD983X_SIGN_OUTPUT_COMPARATOR,
};

enum OutputMode {
  OUTPUT_MODE_SINE        = AD983X_OUTPUT_MODE_SINE,
  OUTPUT_MODE_TRIANGLE    = AD983X_OUTPUT_MODE_TRIANGLE,
};

extern volatile uint16_t tsunami_last_edge;
extern volatile uint16_t tsunami_interval;

class Tsunami_Class {
public:
  Tsunami_Class();
  void begin();

  inline void setFrequencyWord(byte reg, uint32_t frequency) {
    ad983x_set_frequency(&dds, reg, frequency);
  }

  inline void setPhaseWord(byte reg, uint32_t phase) {
    ad983x_set_phase(&dds, reg, phase);
  }

  inline void setSignOutput(SignOutput out) {
    ad983x_set_sign_output(&dds, (ad983x_sign_output_t)out);
  }

  inline void setOutputMode(OutputMode out) {
    ad983x_set_output_mode(&dds, (ad983x_output_mode_t)out);
  }

  // These overloads automatically set the unused frequency register, then
  // switch to it.
  inline void setFrequency(long int frequency) {
    current_reg = 1 - current_reg;
    setFrequency(current_reg, frequency);
    selectFrequency(current_reg);
  }

  inline void setFrequency(double frequency) {
    current_reg = 1 - current_reg;
    setFrequency(current_reg, (float)frequency);
    selectFrequency(current_reg);
  }

  inline void setFrequency(float frequency) {
    current_reg = 1 - current_reg;
    setFrequency(current_reg, frequency);
    selectFrequency(current_reg);
  }

  inline uint32_t computeFrequencyWord(uint32_t frequency) {
    // This is a manual expansion of (frequency * 2^28) / m_frequency_mhz
    // Since it doesn't require 64 bit multiplies or divides, it results in
    // substantially smaller code sizes.
    uint32_t lval = ((frequency & 0xFF) << 22) / (15625l * TSUNAMI_FREQUENCY);
    uint32_t mval = ((frequency & 0xFF00) << 14) / (15625l * TSUNAMI_FREQUENCY);
    uint32_t hval = ((frequency & 0xFF0000) << 6) / (15625l * TSUNAMI_FREQUENCY);
    return (hval << 16) + (mval << 8) + lval;
  }

  inline void setFrequency(byte reg, long int frequency)  {
    setFrequencyWord(reg, computeFrequencyWord(frequency));
  }

  inline void setFrequency(byte reg, double frequency) {
    setFrequencyWord(reg, (frequency * (1l << 28)) / (TSUNAMI_FREQUENCY * 1000000));
  }

  inline void setFrequency(byte reg, float frequency) {
    setFrequencyWord(reg, (frequency * (1l << 28)) / (TSUNAMI_FREQUENCY * 1000000));
  }

  inline void reset(boolean in_reset) {
    digitalWrite(TSUNAMI_DDS_RESET, in_reset);
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
    mcp49xx_write(&dac, TSUNAMI_AMPLITUDE_ID, amp);
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
    mcp49xx_write(&dac, TSUNAMI_OFFSET_ID, off);
  }
  
  mcp49xx_t dac;
private:
  ad983x_t dds;
  uint8_t current_reg;
};

extern Tsunami_Class Tsunami;

#endif
