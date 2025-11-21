#include "matlab_funcs.h"

#define FILTER_MODE 1
#define RTK 1

Vector13 EstimateStateFCN(Vector13 x_est, t_constantsASTRA constantsASTRA, Vector15 z, float dT, float GND, Matrix12_12 &P, bool new_imu_packet, bool new_gps_packet) {
  //// M-EKF Implementation
  // Filter mode (1 for full INS when GPS signals available, 0 for pure
  // integration after launch when no GPS available | limit flight time)

  // Remove bias from gyro
  z.segment<3>(3) = z.segment<3>(3) - x_est.segment<3>(10) * (FILTER_MODE == 1 || GND == 1);

  // Extract quaternion
  Vector12 dx = Vector12::Zero();
  Vector4 q = x_est.segment<4>(0);
  Vector4 t_zseg;
  t_zseg << 0, z.segment<3>(3);
  Vector4 qdot = 0.5 * HamiltonianProd(q) * t_zseg;
  x_est.segment<4>(0) = q + qdot * dT;
  q = x_est.segment<4>(0);

  // A-priori quaternion estimate and rotation matrix
  q.normalize();
  Matrix3_3 R_b2i = quatRot(q).transpose();

  // State Transition Matrix
  Matrix12_12 F = StateTransitionMat(z.segment<3>(0), z.segment<3>(3), R_b2i);

  Vector3 g_vec3;
  g_vec3 << 0, 0, constantsASTRA.g;

  // Propagate rest of state using IMU
  x_est.segment<3>(7) = x_est.segment<3>(7) + (R_b2i * z.segment<3>(0) - g_vec3) * dT;
  x_est.segment<3>(4) = x_est.segment<3>(4) + x_est.segment<3>(7) * dT;

  // Discrete STM
  Matrix12_12 Phi = matrixExpPade6(F * dT);

  // Extract Matrices
  Matrix12_12 Q = constantsASTRA.Q;
  Matrix6_6 R = constantsASTRA.R;

  // Process Noise Covariance and a-priori propagation step
  Q = 0.4 * Q;
  P = Phi * P * Phi.transpose() + Q;

  if (new_imu_packet && (FILTER_MODE == 1 || GND == 1)) {

    // Measurement matrix
    Matrix6_12 H = Matrix6_12::Zero();
    H.block<3, 3>(0, 0) = zetaCross(R_b2i.transpose() * g_vec3);
    H.block<3, 3>(3, 0) = zetaCross(R_b2i.transpose() * constantsASTRA.mag);

    // Measurement Noise Covariance
    float w = 1 + 1e5 * (1 - GND);
    R.block<3, 3>(0, 0) = R.block<3, 3>(0, 0) * w;

    // A priori covariance and Kalman gain

    Matrix12_6 L = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Predicted measurements
    Vector6 z_hat;
    z_hat << R_b2i.transpose() * g_vec3, R_b2i.transpose() * constantsASTRA.mag;

    // Kalman Gain Weighting based on predicted acceleration
    Matrix12_12 ILH = (Matrix12_12::Identity() - L * H);
    P = ILH * P * ILH.transpose() + L * R * L.transpose();
    Vector6 z_slice;
    z_slice << z.segment<3>(0), z.segment<3>(6);
    Vector6 residual = (z_slice - z_hat);
    dx = dx + L * residual;
  }
  if (new_gps_packet && (FILTER_MODE == 1 || GND == 1)) {

    // Measurement matrix
    Matrix6_12 H = Matrix6_12::Zero();
    H.block<3, 3>(0, 3) = Matrix3_3::Identity();
    H.block<3, 3>(3, 6) = Matrix3_3::Identity();

    // Measurement Covariance Matrix
    float gps_pos_covar = 1 * RTK + 10 * (1 - RTK);
    float gps_vel_covar = gps_pos_covar;
    Vector6 R_vec;
    R_vec << gps_pos_covar * gps_pos_covar * Vector3::Ones(), gps_vel_covar * gps_vel_covar * Vector3::Ones();
    Matrix6_6 R = R_vec.asDiagonal();

    // A priori covariance and Kalman gain
    Matrix12_6 L = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Predicted measurements
    Vector6 z_hat;
    z_hat << x_est.segment<3>(4), x_est.segment<3>(7);

    // Kalman Gain Weighting based on predicted acceleration
    Matrix12_12 ILH = (Matrix12_12::Identity() - L * H);
    P = ILH * P * ILH.transpose() + L * R * L.transpose();
    Vector6 residual = (z.segment<6>(9) - z_hat);
    Vector12 inn = L * residual;
    dx = dx + inn;
  }

  if (FILTER_MODE == 1 || GND == 1) {
    // Update full-state estimates
    Vector4 dq;
    dq << 1, dx.segment<3>(0) / 2;
    dq.normalize();

    Vector4 q_nom = HamiltonianProd(q) * dq;
    q_nom.normalize();
    x_est.segment<4>(0) = q_nom;
    x_est.segment<9>(4) = x_est.segment<9>(4) + dx.segment<9>(3);
  }
  x_est.segment<6>(4) = x_est.segment<6>(4) * (1 - GND);
  return x_est;
}
