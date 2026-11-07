// Prints the firmware's conversion for every ADC value as CSV, so
// tools/check_firmware_vs_model.py can compare it with the Python model.
#include <cstdio>
#include "Arduino.h"
#include "config.h"
#include "sensors_analog.h"

int fake_adc[24] = {0};
uint32_t fake_millis = 0;
long fake_analog_reads = 0;

int main() {
  printf("adc,fsr_N,therm_C,therm_fault\n");
  for (int adc = 0; adc <= ADC_MAX; adc++) {
    float t = analog_math::thermCFromOhms(analog_math::thermOhmsFromAdc((float)adc));
    printf("%d,%.6f,%.6f,%d\n", adc,
           analog_math::fsrNewtonsFromAdc((float)adc), t,
           analog_math::thermAdcIsFault((float)adc) ? 1 : 0);
  }
  return 0;
}
