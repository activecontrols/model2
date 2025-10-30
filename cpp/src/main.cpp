#include <Arduino.h>
#include "matlab_funcs.hpp"

void setup() {
  Serial.begin(115200);
  Serial.println("Connected - starting benchmark");

  Vector13 x_est;
  t_constantsASTRA constantsASTRA;
  Vector15 z;
  double dT;
  double GND;

  Matrix12_12 P = 1 * Matrix12_12::Identity();

  bool new_imu_packet;
  bool new_gps_packet;

  EstimateStateFCN(x_est, constantsASTRA, z, dT, GND, P, new_imu_packet, new_gps_packet);
}

void loop() {
}