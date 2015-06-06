#include <avr/boot.h>
#include <SPI.h>
#include <tsunami.h>

#define LED_PIN 13
#define RUN_REPEATEDLY true

char err[255];

typedef bool (*testfunction)(void);

typedef struct {
  char *name;
  testfunction func;
} test_t;

void log_value(char *name, int32_t value) {
  char buf[32];
  sprintf(buf, "# %s = %ld", name, value);
  Serial.println(buf);
}

void log_value(char *name, double value) {
  char buf[32];
  sprintf(buf, "# %s = %f", name, value);
  Serial.println(buf);
}

bool _assert_nearly_equal(char *name, double value, double target, double error) {
  log_value(name, value);
  if(value < target - error || value > target + error) {
    sprintf(err, "%s: %f is not between %f and %f", name, value, target - error, target + error);
    return false;
  }
  return true;
}

bool _assert_nearly_equal(char *name, int32_t value, int32_t target, int32_t error) {
  log_value(name, value);
  if(value < target - error || value > target + error) {
    sprintf(err, "%s: %ld is not between %ld and %ld", name, value, target - error, target + error);
    return false;
  }
  return true;
}

bool _assert_nearly_equal(char *name, int16_t value, int16_t target, int16_t error) {
  return _assert_nearly_equal(name, (int32_t)value, (int32_t)target, (int32_t)error);
}

#define assert_nearly_equal(value, min, max, errmsg) if(!_assert_nearly_equal(value, min, max, errmsg)) return false

bool read_usb_id() {
  Serial.print("# usb_id = ");
  for(int i = 0; i < 10; i++)
    Serial.print(boot_signature_byte_get((0x07 * 2) + i), HEX);
  Serial.println();
  return true;  
}

bool test_zero_offset() {
  Tsunami.measureCurrentVoltage();
  delay(200);
  assert_nearly_equal("input_offset", Tsunami.measureCurrentVoltage(), 0, 200);
  return true;
}

bool test_offset_set() {
  delay(200);
  int offset = Tsunami.measureCurrentVoltage();
  Tsunami.setOffset(-1000);
  delay(200);
  assert_nearly_equal("output_offset_neg", Tsunami.measureCurrentVoltage(), offset - 1000, 200);
  Tsunami.setOffset(1000);
  delay(200);
  assert_nearly_equal("output_offset_pos", Tsunami.measureCurrentVoltage(), offset + 1000, 200);
  return true;
}

bool test_amplitude_set() {
  Tsunami.setFrequency(32768);
  Tsunami.setAmplitude(6000);
  delay(500);
  // Temporary values until we get production boards
  assert_nearly_equal("input_amp_6", Tsunami.measurePeakVoltage(), 3300, 100);

  // TODO: Figure out why we don't get consistent values for this reading
  Tsunami.setAmplitude(1000);
  pinMode(TSUNAMI_PEAK, OUTPUT);
  pinMode(TSUNAMI_PEAK, INPUT);
  delay(1000);
  assert_nearly_equal("input_amp_1", Tsunami.measurePeakVoltage(), 0, 3300);

  Tsunami.setAmplitude(0);
  delay(500);
  assert_nearly_equal("input_amp_0", Tsunami.measurePeakVoltage(), -3130, 150);

  return true;
}

bool test_freq_phase() {
  Tsunami.setAmplitude(6000);
  Tsunami.setFrequency(1000.0);
  delay(100);
  assert_nearly_equal("input_freq_1k", (int32_t)Tsunami.measureFrequency(), 1000l, 2l);
  assert_nearly_equal("input_phase_1k", (int16_t)(1000 * Tsunami.measurePhase()), 1000, 1);

  Tsunami.setFrequency(100000.0);
  delay(100);
  assert_nearly_equal("input_freq_100k", (int32_t)Tsunami.measureFrequency(), 100000l, 2l);
  assert_nearly_equal("input_phase_100k", (int16_t)(1000 * Tsunami.measurePhase()), 1000, 1);

  Tsunami.setFrequency(1000000.0);
  // Reset phase counter for sensitive measurement
  Tsunami.reset(true); Tsunami.reset(false);
  delay(100);
  assert_nearly_equal("input_freq_1m", (int32_t)Tsunami.measureFrequency(), 1000000l, 2l);
  assert_nearly_equal("input_phase_1m", (int16_t)(1000 * Tsunami.measurePhase()), 730, 50);
}

test_t tests[] = {
  {"Read USB ID", &read_usb_id},
  {"Test zero offset", &test_zero_offset},
  {"Test offset adjustment", &test_offset_set},
  {"Test amplitude adjustment", &test_amplitude_set},
  {"Test frequency and phase", &test_freq_phase},
  {NULL, NULL}
};

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(2000);
}

int run_tests() {
#if RUN_REPEATEDLY
  // Persist any errors we see between runs
  static int first_failure = 0;
#else
  int first_failure = 0;
#endif

  for(int i = 0; tests[i].func != NULL; i++) {    
    Tsunami.begin();
    
    bool result = tests[i].func();
    Serial.print(tests[i].name);
    Serial.print("... ");
    if(!result) {
      Serial.println("FAIL");
      Serial.println(err);
      if(first_failure == 0)
        first_failure = i + 1;
    } else {
      Serial.println("OK");
    }
  }
  return first_failure;
}

void error_code(int code) {
  while(1) {
    unsigned long when;
    for(int i = 0; i < code * 2; i++) {
      digitalWrite(LED_PIN, i & 1);
      when = millis() + 500;
      while(millis() < when) {
        if(Serial.read() != -1)
          return;
      }
    }
    
    // Longer delay at the end
    when = millis() + 3000;
    while(millis() < when) {
      if(Serial.read() != -1)
        return;
    }
    
#if RUN_REPEATEDLY
    return;
#endif
  }
}

void loop() {
  Serial.println("Executing test suite...");
  int result = run_tests();
  if(result == 0) {
    Serial.println("All tests passed.");
    digitalWrite(LED_PIN, HIGH);
  }
  
  Serial.println("Press enter to rerun test suite.");
  // Flash error code while waiting for input
  error_code(result);
  Serial.println();
}

