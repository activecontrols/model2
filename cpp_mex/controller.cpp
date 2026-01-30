#include "controller.h"
#include "matlab_funcs.h"

namespace Controller {
constantsASTRA_t constantsASTRA;
Matrix18_18 P;
Matrix9_9 Flight_P;
Vector13 x_est;
Vector3 lastEMA;
Matrix9_4 dnf_X;
Matrix9_4 dnf_Y;
float last_thrust;
bool last_GND;

unsigned long last_call_time; // ms

void begin() {
  reset_controller_state();
}

void reset_controller_state() {
  constantsASTRA.g = 9.8015;
  constantsASTRA.m = 1.2490;
  constantsASTRA.mag << -0.4512, 0, 0.8924;
  constantsASTRA.Q << 2.500083e-06, 0, 0, 0, 0, 0, 0, 0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, //
      0, 2.500083e-06, 0, 0, 0, 0, 0, 0, 0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0,                 //
      0, 0, 2.500083e-06, 0, 0, 0, 0, 0, 0, 0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0,                 //
      0, 0, 0, 2.083349e-09, 0, 0, 6.250078e-07, 0, 0, 0, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0,      //
      0, 0, 0, 0, 2.083349e-09, 0, 0, 6.250078e-07, 0, 0, 0, 0, 0, -2.083333e-09, 0, 0, 0, 0,      //
      0, 0, 0, 0, 0, 2.083349e-09, 0, 0, 6.250078e-07, 0, 0, 0, 0, 0, -2.083333e-09, 0, 0, 0,      //
      0, 0, 0, 6.250078e-07, 0, 0, 2.500042e-04, 0, 0, 0, 0, 0, -1.250000e-06, 0, 0, 0, 0, 0,      //
      0, 0, 0, 0, 6.250078e-07, 0, 0, 2.500042e-04, 0, 0, 0, 0, 0, -1.250000e-06, 0, 0, 0, 0,      //
      0, 0, 0, 0, 0, 6.250078e-07, 0, 0, 2.500042e-04, 0, 0, 0, 0, 0, -1.250000e-06, 0, 0, 0,      //
      -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 5.000000e-06, 0, 0, 0, 0, 0, 0, 0, 0,                 //
      0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 5.000000e-06, 0, 0, 0, 0, 0, 0, 0,                 //
      0, 0, -2.500000e-07, 0, 0, 0, 0, 0, 0, 0, 0, 5.000000e-06, 0, 0, 0, 0, 0, 0,                 //
      0, 0, 0, -1.250000e-06, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0, 2.500000e-05, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, -1.250000e-06, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0, 2.500000e-05, 0, 0, 0, 0,     //
      0, 0, 0, 0, 0, -1.250000e-06, 0, 0, -2.083333e-09, 0, 0, 0, 0, 0, 2.500000e-05, 0, 0, 0,     //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-06, 0, 0,                             //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-06, 0,                             //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.500000e-06;                             //

  constantsASTRA.R = Matrix6_6::Zero();
  constantsASTRA.R.block<3, 3>(0, 0) = Matrix3_3::Identity() * 0.100;
  constantsASTRA.R.block<3, 3>(3, 3) = Matrix3_3::Identity() * 0.100;

  constantsASTRA.K_Att << 1.341480e+00, -4.423127e-16, -2.263115e-17, 1.949556e-01, -3.652184e-17, -5.666956e-17, -6.582806e-01, 5.461372e-16, -2.780114e-16, //
      4.924545e-16, 1.341480e+00, -3.901911e-16, 5.099383e-17, 1.949556e-01, -3.192517e-16, -2.775558e-16, -6.582806e-01, -2.514962e-16,                      //
      -4.006925e-16, 5.609321e-16, 4.035990e+00, -5.718917e-17, 1.014174e-16, 2.059154e+00, 3.453548e-16, -4.435380e-16, -1.581139e+00;                       //
                                                                                                                                                              //

  P = 1 * Matrix18_18::Identity();
  x_est = Vector13::Zero();
  x_est[0] = 1;
  lastEMA = Vector3::Zero();
  dnf_X = Matrix9_4::Ones();
  dnf_Y = Matrix9_4::Ones();
  last_thrust = 0;
  last_GND = true;

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

  if (!ci.GND_val && last_GND) { // we left GND this frame
    Flight_P = P.block<9, 9>(0, 0);
  }

  if (ci.GND_val) {
    x_est = GroundEstimator(x_est, constantsASTRA, z, dT, P, ci.new_imu_packet, ci.new_gps_packet);
  } else {
    x_est = FlightEstimator(x_est, constantsASTRA, z, dT, Flight_P, ci.new_gps_packet);
  }

  Vector3 EMA_G = EMA_Gyros(z, lastEMA);
  Vector16 X = StateAUG(x_est, EMA_G);
  Vector3 TargetPos;
  TargetPos << ci.target_pos_north, ci.target_pos_west, ci.target_pos_up;
  Vector4 raw_co = ASTRAv2_Controller(TargetPos, X, constantsASTRA, dT);
  if (ci.GND_val) {
    raw_co = Vector4::Zero();
  }
  last_thrust = raw_co(2);
  last_GND = ci.GND_val;

  Controller_Output co;
  co.gimbal_yaw_deg = raw_co(0) * 180 / M_PI;
  co.gimbal_pitch_deg = raw_co(1) * 180 / M_PI;
  co.thrust_N = raw_co(2);
  co.roll_rad_sec_squared = raw_co(3);
  return co;
}
} // namespace Controller