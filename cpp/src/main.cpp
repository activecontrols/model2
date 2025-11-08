#include <Arduino.h>
#include "matlab_funcs.hpp"
#include "sample_data.hpp" // contains x_est_arr, z_arr, covar_arr, dT_arr, GND_arr, exp_x_est_arr

void setup() {
  extern uint8_t SetSysClock_PLL_HSE(uint8_t bypass, bool lowspeed);
  SetSysClock_PLL_HSE(1, (bool)false);

  delay(5000);
  Serial.begin(115200);
  Serial.println("Connected - starting astra sim");

  t_constantsASTRA constantsASTRA;
  constantsASTRA.g = 9.8100;
  constantsASTRA.mag << 0.8660, 0, -0.5000;
  constantsASTRA.Q << 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, 0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, 0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10,
      0, 0, 0, 6.250000e-07, 0, 0, 2.500000e-04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000e-07, 0, 0, 2.500000e-04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6.250000e-07, 0, 0, 2.500000e-04, 0, 0, 0, 0, 0, 0,
      2.083333e-09, 0, 0, 6.250000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.083333e-09, 0, 0, 6.250000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.083333e-09, 0, 0, 6.250000e-07, 0, 0, 0, -6.250000e-10, 0, 0, 0, 0, 0,
      0, 0, 0, 1.250000e-07, 0, 0, 0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07, 0, 0, 0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07;
  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.1500;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.1000;

  long long start_t = millis();
  Matrix12_12 P = 1 * Matrix12_12::Identity();
  Vector15 lastZ = Vector15::Zero();

  // Loop over all timesteps
  for (int idx = 0; idx < MAX_IDX; idx++) {

    // Construct Eigen vectors directly from arrays
    Vector13 x_est(x_est_arr[idx]);
    Vector15 z(z_arr[idx]);
    float dT_val = 0.001;
    float GND_val = GND_arr[idx];

    Vector15 temp_z = z;
    temp_z.segment<3>(3) = temp_z.segment<3>(3) - x_est.segment<3>(10);
    bool new_imu_packet = (lastZ.segment<9>(0) - temp_z.segment<9>(0)).sum() != 0;
    bool new_gps_packet = (lastZ.segment<6>(9) - temp_z.segment<6>(9)).sum() != 0;

    // Serial.print(idx);
    // Serial.print(" imu: ");
    // Serial.print(new_imu_packet);
    // Serial.print(" gps: ");
    // Serial.println(new_gps_packet);

    Vector13 ret_state = EstimateStateFCN(x_est, constantsASTRA, z, dT_val, GND_val, P, new_imu_packet, new_gps_packet);
    lastZ = z;

    // comparison with expected output
    for (int i = 0; i < 13; i++) {
      if (abs(ret_state(i) - exp_x_est_arr[idx][i]) > 0.0002) {
        Serial.print("Mismatch at idx: ");
        Serial.print(idx);
        Serial.print(" element: ");
        Serial.print(i);
        Serial.print(" expected: ");
        Serial.print(exp_x_est_arr[idx][i], 6);
        Serial.print(" got: ");
        Serial.println(ret_state(i), 6);
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

void loop() {
  for (;;) {
    Serial.println("Done!");
    delay(100000);
  }
}
