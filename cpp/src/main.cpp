#include <Arduino.h>
#include "matlab_funcs.hpp"
#include "sample_data.hpp"

void setup() {
  Serial.begin(115200);
  Serial.println("Connected - starting benchmark");

  Vector12 x_est;
  t_constantsASTRA constantsASTRA;
  Vector15 z;
  Vector3 covar_vec;
  double dT;
  Matrix12_12 Q;
  double GND;

  Matrix12_12 P = 1 * Matrix12_12::Identity();

  bool new_imu_packet;
  bool new_gps_packet;

  EstimateStateFCN(x_est, constantsASTRA, z, covar_vec, dT, Q, GND, P, new_imu_packet, new_gps_packet);
}

void loop() {
}
