#include <Arduino.h>
#include "matlab_funcs.h"
#include "filter_sample_data.h"

void setup_filter() {
  extern uint8_t SetSysClock_PLL_HSE(uint8_t bypass, bool lowspeed);
  SetSysClock_PLL_HSE(1, (bool)false);

  delay(5000);
  Serial.begin(115200);
  Serial.println("Connected - starting filter sim");

  long long start_t = millis();

  Matrix9_4 X;
  Matrix9_4 Y;
  Vector9 pre_sample(filter_in_arr[0]);
  for (int i = 0; i < 4; i++) {
    X.block<9, 1>(0, i) = pre_sample;
    Y.block<9, 1>(0, i) = pre_sample;
  }

  // Loop over all timesteps
  for (int idx = 0; idx < MAX_IDX_FILTER; idx++) {
    Vector9 filter_in(filter_in_arr[idx]);
    Vector9 filter_out = DigitalNF(filter_in, 0.0, 68.0, 0.001, X, Y);

    // comparison with expected output
    for (int i = 0; i < 9; i++) {
      if (abs(filter_out(i) - filter_out_arr[idx][i]) > 0.0002) {
        Serial.print("Mismatch at idx: ");
        Serial.print(idx);
        Serial.print(" element: ");
        Serial.print(i);
        Serial.print(" expected controller output: ");
        Serial.print(filter_out_arr[idx][i], 6);
        Serial.print(" got: ");
        Serial.println(filter_out(i), 6);
      }
    }
  }

  long long end_t = millis();

  Serial.print("Finished astra sim in ");
  Serial.print(end_t - start_t);
  Serial.println(" ms.");
  Serial.println(SystemCoreClock);
  Serial.println(HAL_RCC_GetHCLKFreq()); // Expect ~480000000 Hz
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    Serial.println("Running on M7 core");
  } else {
    Serial.println("Running on M4 core");
  }
}

void loop_filter() {
  for (;;) {
    Serial.println("Done!");
    delay(100000);
  }
}
