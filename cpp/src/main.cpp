#include <Arduino.h>
#include "matlab_funcs.h"
#include "sample_data.h" //contains Z, TargetPos, GND, and expected controller output

void setup() {
  extern uint8_t SetSysClock_PLL_HSE(uint8_t bypass, bool lowspeed);
  SetSysClock_PLL_HSE(1, (bool)false);

  delay(5000);
  Serial.begin(115200);
  Serial.println("Connected - starting astra sim");

  t_constantsASTRA constantsASTRA;
  constantsASTRA.g = 9.8100;
  constantsASTRA.m = 1;
  constantsASTRA.mag << 0.8660, 0, -0.5000;
  constantsASTRA.Q << 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, //
      0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0,                 //
      0, 0, 1.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10,                 //
      0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 6.250000e-07, 0, 0, 2.083333e-09, 0, 0, 0,                  //
      0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 2.500000e-04, 0, 0, 6.250000e-07, 0, 0, 0,                  //
      -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07, 0, 0,                 //
      0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07, 0,                 //
      0, 0, -6.250000e-10, 0, 0, 0, 0, 0, 0, 0, 0, 1.250000e-07;                 //
  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.1500;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.1000;

  Matrix4_12 K;
  K << 1.727316e+00, 1.397551e-15, -4.496957e-16, -2.191563e-17, -9.946226e-05, 2.443746e-18, 1.009896e-16, -1.808889e-01, -9.204288e-17, 2.101464e-01, 9.046160e-17, -2.504290e-16,   //
      -1.001805e-15, 1.752664e+00, -1.979585e-14, 9.946226e-05, -3.640769e-18, 1.341432e-18, 1.808896e-01, 1.090076e-16, 1.485138e-16, -8.495741e-17, 2.163589e-01, -4.800818e-15,     //
      2.068573e-14, -1.808309e-13, -7.941772e-14, -1.782415e-17, -7.805697e-17, 4.559014e-03, -2.450348e-14, -1.100699e-14, 3.040842e+00, -1.390689e-15, -2.931921e-15, -2.405146e-14, //
      -3.030093e-15, 1.483349e-14, 5.065571e+00, 4.278588e-18, -5.374655e-18, 2.685444e-18, 2.056540e-15, 1.574826e-16, 2.541668e-16, -1.544970e-16, 9.234013e-16, 2.252961e+00;       //

  long long start_t = millis();
  Matrix12_12 P = 1 * Matrix12_12::Identity();
  Vector15 lastZ = Vector15::Zero();
  Vector13 x_est = Vector13::Zero();
  x_est[0] = 1;
  Vector3 lastEMA = Vector3::Zero();

  Matrix9_6 dnf_X = Matrix9_6::Ones();
  Matrix9_6 dnf_Y = Matrix9_6::Ones();
  float last_thrust = constantsASTRA.g * constantsASTRA.m;

  // Loop over all timesteps
  for (int idx = 0; idx < MAX_IDX; idx++) {
    // Construct Eigen vectors directly from arrays
    Vector15 z(z_arr[idx]);
    float GND_val = GND_arr[idx];
    Vector3 TargetPos(target_pos_arr[idx]);

    Vector9 imu = z.segment<9>(0);
    Vector9 filt_imu = DigitalNF(imu, GND_val, last_thrust, dT, dnf_X, dnf_Y);
    z.segment<9>(0) = filt_imu;

    Vector15 temp_z = z;
    temp_z.segment<3>(3) = temp_z.segment<3>(3) - x_est.segment<3>(10);
    bool new_imu_packet = (lastZ.segment<9>(0) - temp_z.segment<9>(0)).sum() != 0;
    bool new_gps_packet = (lastZ.segment<6>(9) - temp_z.segment<6>(9)).sum() != 0;

    x_est = EstimateStateFCN(x_est, constantsASTRA, z, dT, GND_val, P, new_imu_packet, new_gps_packet);
    Vector3 EMA_G = EMA_Gyros(z, lastEMA);
    Vector15 X = StateAUG(x_est, EMA_G);
    Vector12 error = ref_generator3(X, TargetPos);
    Vector4 raw_co = -K * error;
    raw_co(2) = raw_co(2) + constantsASTRA.g * constantsASTRA.m;
    raw_co = output_clamp(raw_co);
    if (GND_val) {
      raw_co = Vector4::Zero();
    }

    last_thrust = raw_co(2);
    lastZ = z;

    // comparison with expected output
    for (int i = 0; i < 4; i++) {
      if (abs(raw_co(i) - exp_controller_output[idx][i]) > 0.0002) {
        Serial.print("Mismatch at idx: ");
        Serial.print(idx);
        Serial.print(" element: ");
        Serial.print(i);
        Serial.print(" expected controller output: ");
        Serial.print(exp_controller_output[idx][i], 6);
        Serial.print(" got: ");
        Serial.println(raw_co(i), 6);
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
