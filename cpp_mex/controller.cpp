#include "controller.h"
#include "matlab_funcs.h"

namespace Controller {
t_constantsASTRA constantsASTRA;
Matrix12_12 P;
Vector13 x_est;
Vector3 lastEMA;
Matrix9_4 dnf_X;
Matrix9_4 dnf_Y;
float last_thrust;

unsigned long last_call_time; // ms

void begin() {
  reset_controller_state();
}

void reset_controller_state() {
  constantsASTRA.g = 9.8015;
  constantsASTRA.m = 1.2490;
  constantsASTRA.mag << -0.4512, 0, 0.8924;
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

  P = 1 * Matrix12_12::Identity();
  x_est = Vector13::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();
  dnf_X = Matrix9_4::Ones();
  dnf_Y = Matrix9_4::Ones();
  last_thrust = 0;

  ASTRAv2_Controller_reset();

  last_call_time = micros();
}

Controller_Output get_controller_output(Controller_Input ci) {
  unsigned long call_time = micros();
  float dT = (call_time - last_call_time) / 1000000.0;
  last_call_time = call_time;

  Vector15 z;
  // clang-format off
  z << ci.accel_x, ci.accel_y, ci.accel_z, 
       ci.gyro_yaw, ci.gyro_pitch, ci.gyro_roll,
       ci.mag_x, ci.mag_y, ci.mag_z,
       ci.gps_pos_north, ci.gps_pos_west, ci.gps_pos_up, 
       ci.gps_vel_north, ci.gps_vel_west, ci.gps_vel_up;
  // clang-format on

  Vector9 imu = z.segment<9>(0);
  Vector9 filt_imu = DigitalNF(imu, ci.GND_val, last_thrust, dT, dnf_X, dnf_Y);
  z.segment<9>(0) = filt_imu;

  x_est = EstimateStateFCN(x_est, constantsASTRA, z, dT, ci.GND_val, P, ci.new_imu_packet, ci.new_gps_packet);
  Vector3 EMA_G = EMA_Gyros(z, lastEMA);
  Vector16 X = StateAUG(x_est, EMA_G);
  Vector3 TargetPos;
  TargetPos << ci.target_pos_north, ci.target_pos_west, ci.target_pos_up;
  Vector4 raw_co = ASTRAv2_Controller(TargetPos, X, constantsASTRA, dT);
  if (ci.GND_val) {
    raw_co = Vector4::Zero();
  }
  last_thrust = raw_co(2);

  Controller_Output co;
  co.gimbal_yaw_deg = raw_co(0) * 180 / M_PI;
  co.gimbal_pitch_deg = raw_co(1) * 180 / M_PI;
  co.thrust_N = raw_co(2);
  co.roll_rad_sec_squared = raw_co(3);
  return co;
}
} // namespace Controller