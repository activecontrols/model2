#include <Arduino.h>
#include "matlab_funcs.hpp"
#include "sample_data.hpp"  // contains x_est_arr, z_arr, covar_arr, dT_arr, GND_arr, exp_x_est_arr

void setup() {
    Serial.begin(115200);
    Serial.println("Connected - starting astra sim");

    t_constantsASTRA constantsASTRA;
  
    /* NOT SURE IF THIS IS NECESSARY OR DONE CORRECTLY??  
    constantsASTRA.m = 1.0;        
    constantsASTRA.l = 1.0;        
    constantsASTRA.g = 9.81;       
    constantsASTRA.rTB = 0.1;      
    constantsASTRA.j = Matrix3_3::Identity(); 
    constantsASTRA.T = 1.0;        
    constantsASTRA.mag << 0, 0, 1; 
    */
  
    // Covariance matrix (estimated) (makes more sense to use a per-step Q but I'm not sure how to find that...)
    Matrix12_12 Q_matrix = Matrix12_12::Zero();
      // Quaternion noise
      Q_matrix(0,0) = 1e-6;
      Q_matrix(1,1) = 1e-6;
      Q_matrix(2,2) = 1e-6;
      
      // Position noise
      Q_matrix(3,3) = 1e-4;
      Q_matrix(4,4) = 1e-4;
      Q_matrix(5,5) = 1e-4;
      
      // Velocity noise
      Q_matrix(6,6) = 1e-3;
      Q_matrix(7,7) = 1e-3;
      Q_matrix(8,8) = 1e-3;
      
      // Gyro bias noise
      Q_matrix(9,9)   = 1e-8;
      Q_matrix(10,10) = 1e-8;
      Q_matrix(11,11) = 1e-8;
  
    long long start_t = millis();

    bool new_imu_packet;     //NOT SURE HOW TO INITIALIZE THESE...
    bool new_gps_packet;

    // Loop over all timesteps
    for (int idx = 0; idx < MAX_IDX; idx++) {

        // Construct Eigen vectors directly from arrays
        Vector12 x_est(x_est_arr[idx]);
        Vector15 z(z_arr[idx]);
        Vector3 covar_vec(covar_arr[idx]);

        double dT_val = dT_arr[idx];
        double GND_val = GND_arr[idx];

        Vector12 ret_state = EstimateStateFCN(x_est, constantsASTRA, z, covar_vec, dT_val, Q_matrix, GND_val, new_imu_packet, new_gps_packet);

        // comparison with expected output
        for (int i = 0; i < 12; i++) {
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
}

void loop() {
}
