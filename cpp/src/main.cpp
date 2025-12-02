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
  constantsASTRA.g = 9.8015;
  constantsASTRA.m = 1.2490;
  constantsASTRA.mag << 0.8660, 0, -0.5000;
  constantsASTRA.Q << 4.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0, 0, //
      0, 4.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10, 0,                 //
      0, 0, 4.000004e-05, 0, 0, 0, 0, 0, 0, 0, 0, -6.250000e-10,                 //
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
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.1200;

  constantsASTRA.K_Att << 1.133604e+00, -3.297985e-16, 1.683627e-16, 1.925898e-01, -3.507285e-17, -7.648992e-17, -6.324555e-01, 6.628460e-17, -1.142330e-16, //
      2.717801e-16, 1.136934e+00, 2.449270e-16, 4.799793e-17, 1.985662e-01, 2.505480e-17, -7.524250e-17, -6.324555e-01, -2.397136e-16,                       //
      -1.788593e-15, 1.828852e-15, 8.462398e+00, -2.226297e-16, 2.620046e-16, 4.355001e+00, 1.286862e-15, -1.097175e-15, -4.472136e+00;                      //

  long long start_t = millis();
  Matrix12_12 P = 1 * Matrix12_12::Identity();
  Vector15 lastZ = Vector15::Zero();
  Vector13 x_est = Vector13::Zero();
  x_est[0] = 1;
  Vector3 lastEMA = Vector3::Zero();

  Matrix9_4 dnf_X = Matrix9_4::Ones();
  Matrix9_4 dnf_Y = Matrix9_4::Ones();
  float last_thrust = constantsASTRA.g * constantsASTRA.m;
  float allowed_err[4] = {0.0002, 0.0002, 0.002, 0.0002};
  ASTRAv2_Controller_reset();

  // Loop over all timesteps
  for (int idx = 0; idx < MAX_IDX; idx++) {
    // Construct Eigen vectors directly from arrays
    Vector15 z(z_arr[idx]);
    float GND_val = GND_arr[idx];
    Vector3 TargetPos(target_pos_arr[idx]);

    Vector9 imu = z.segment<9>(0);
    Vector9 filt_imu = DigitalNF(imu, GND_val, last_thrust, 0.002, dnf_X, dnf_Y);
    z.segment<9>(0) = filt_imu;

    Vector15 temp_z = z;
    temp_z.segment<3>(3) = temp_z.segment<3>(3) - x_est.segment<3>(10);
    bool new_imu_packet = (lastZ.segment<9>(0) - temp_z.segment<9>(0)).sum() != 0;
    bool new_gps_packet = (lastZ.segment<6>(9) - temp_z.segment<6>(9)).sum() != 0;

    x_est = EstimateStateFCN(x_est, constantsASTRA, z, dT, GND_val, P, new_imu_packet, new_gps_packet);
    Vector3 EMA_G = EMA_Gyros(z, lastEMA);
    Vector15 X = StateAUG(x_est, EMA_G);
    Vector4 raw_co = ASTRAv2_Controller(TargetPos, X, constantsASTRA, dT);

    if (GND_val) {
      raw_co = Vector4::Zero();
    }

    last_thrust = raw_co(2);
    lastZ = z;

    // comparison with expected output
    for (int i = 0; i < 4; i++) {
      if (abs(raw_co(i) - exp_controller_output[idx][i]) > allowed_err[i]) {
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
