#include <Arduino.h>
#include "matlab_funcs.hpp"
#include "sample_data.hpp"  //contains Z, TargetPos, GND, and expected controller output

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
  
  Matrix4_12 K << 1.221739e+00, 2.656070e-16, 1.605792e-14, 5.754042e-14, 1.138010e-15, 1.240890e+00, -6.956013e-14, 1.577365e-14, -3.842269e-15, -5.030489e-16, -1.047017e-14, 4.748973e+00, -1.776182e-18,
      5.304654e-05, -5.286303e-17, 1.334778e-18, -5.304654e-05, -1.777516e-19, -1.077854e-17, 9.473637e-18, -4.322925e-18, 7.801908e-19, 4.559014e-03, 6.725418e-19, 2.964490e-17, 9.648303e-02,
      -1.028685e-15, 2.688724e-15, -9.648249e-02, 6.718166e-17, -4.583739e-15, -7.161693e-15, -1.793211e-17, -9.218939e-18, 2.281506e+00, 1.992377e-17, 1.971226e-01, 1.725272e-16, 3.380334e-16,
      5.853777e-15, 1.182562e-16, 2.033502e-01, -1.370787e-14, -5.543101e-16, -3.093029e-16, -4.973344e-17, -4.785263e-16, 5.364513e-01;

  long long start_t = millis();
  Matrix12_12 P = 1 * Matrix12_12::Identity();
  Vector15 lastZ = Vector15::Zero();
  Vector13 x_est = Vector13::Zero();
  x_est[0] = 1;
  Vector3 lastEMA = Vector3::Zero();
  
  // Loop over all timesteps
  for (int idx = 0; idx < MAX_IDX; idx++) {

    // Construct Eigen vectors directly from arrays
    Vector15 z(z_arr[idx]);
    float dT_val = 0.001;
    float GND_val = GND_arr[idx];

    Vector15 temp_z = z;
    temp_z.segment<3>(3) = temp_z.segment<3>(3) - x_est.segment<3>(10);
    bool new_imu_packet = (lastZ.segment<9>(0) - temp_z.segment<9>(0)).sum() != 0;
    bool new_gps_packet = (lastZ.segment<6>(9) - temp_z.segment<6>(9)).sum() != 0;

    //NEEDS TO BE CHANGED TO ALIGN WITH THE NAME OF THE ACTUAL ARRAY IN THE EXPORTED MATLAB DATA
    Vector3 TargetPos(target_pos_arr[idx]); 
    
    // Serial.print(idx);
    // Serial.print(" imu: ");
    // Serial.print(new_imu_packet);
    // Serial.print(" gps: ");
    // Serial.println(new_gps_packet);

    x_est = EstimateStateFCN(x_est, constantsASTRA, z, dT_val, GND_val, P, new_imu_packet, new_gps_packet);
    Vector3 EMA_G = EMA_Gyros(z, lastEMA);
    Vector15 X = StateAUG(x_est, EMA_G);
    Vector12 error = ref_generator3(X, TargetPos);
    Vector4 raw_co = -K * error;
    raw_co = output_clamp(raw_co);

    lastZ = z;
    lastEMA = EMA_G;

    // still need to convert raw_co to the proper controller output??
    Vector4 controller_out = raw_co;  
      
    // comparison with expected output
    for (int i = 0; i < 4; i++) {
      if (abs(controller_out(i) - exp_output_arr[idx][i]) > 0.0002) {     //NOTE THAT ARRAY NAME FOR EXPECTED OUTPUT MIGHT NEED TO BE CHANGED
        Serial.print("Mismatch at idx: ");
        Serial.print(idx);
        Serial.print(" element: ");
        Serial.print(i);
        Serial.print(" expected controller output: ");
        Serial.print(exp_output_arr[idx][i], 6);              // ARRAY NAME NEEDS TO BE UPDATED TO ALIGN WITH MATLAB EXPORT
        Serial.print(" got: ");
        Serial.println(controller_out(i), 6);
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
