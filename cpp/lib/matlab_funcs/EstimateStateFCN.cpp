#include "matlab_funcs.hpp"

Vector12 EstimateStateFCN(Vector12 x_est, t_constantsASTRA constantsASTRA, Vector15 z, Vector3 covar_vec, double dT, Matrix12_12 Q, double GND, Matrix12_12 P, bool new_imu_packet, bool new_gps_packet) {
  //// M-EKF Implementation
  // Propagate nominal state (DEAD-RECKONING PROPAGATION FOR Q, R, AND V)
  // Extract Quaternion
  // REMOVE RATES FROM STATE_VEC, AIM FOR FULL STATE SIM

  // Remove bias from gyro
  z.segment<3>(3) -= x_est.segment<3>(9);

  // Extract quaternion
  Vector12 dx = Vector12::Zero();
  double q0 = sqrt(abs(1 - x_est.segment<3>(0).transpose() * x_est.segment<3>(0)));
  Vector4 q;
  q << q0, x_est.segment<3>(0);
  Matrix4_4 M = HamiltonianProd(q);

  Vector4 t_zseg;
  t_zseg << 0, z.segment<3>(3);

  Vector4 qdot = 0.5 * M * t_zseg;
  Vector3 q_123_dot = qdot.segment<3>(1);

  x_est.segment<3>(0) = q.segment<3>(1) + q_123_dot * dT;

  // A-priori quaternion estimate and rotation matrix
  q0 = sqrt(1 - x_est.segment<3>(0).transpose() * x_est.segment<3>(0));
  q << q0, x_est.segment<3>(0);
  Matrix3_3 R_b2i = quatRot(q).transpose();

  // State Transition Matrix
  Matrix12_12 F = StateTransitionMat(z.segment<3>(0), z.segment<3>(3), R_b2i);

  Vector3 g_vec3;
  g_vec3 << 0, 0, constantsASTRA.g;

  // Propagate rest of state using IMU
  x_est.segment<3>(6) = x_est.segment<3>(6) + (R_b2i * z.segment<3>(0) - g_vec3) * dT;
  x_est.segment<3>(3) = x_est.segment<3>(3) + x_est.segment<3>(6) * dT;

  // Discrete STM
  Matrix12_12 Phi = matrixExpPade6(F * dT);

  // Process Noise Covariance and a-priori propagation step
  Q = 0.4 * Q;
  P = Phi * P * Phi.transpose() + Q;

  if (new_imu_packet) {
    // Measurement matrix
    Matrix6_12 H = Matrix6_12::Zero();

    H.block<3, 3>(0, 0) = zetaCross(R_b2i.transpose() * g_vec3);
    H.block<3, 3>(3, 0) = zetaCross(R_b2i.transpose() * constantsASTRA.mag);

    // Measurement Noise Covariance
    double w = 1 + 300 * (1 - GND);
    Vector6 R_vec;
    R_vec << (covar_vec[0] * w) * (covar_vec[0] * w) * Vector3::Ones(), covar_vec[2] * covar_vec[2] * Vector3::Ones();
    Matrix6_6 R = R_vec.asDiagonal();

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
  if (new_gps_packet) {
    // Measurement matrix
    Matrix6_12 H = Matrix6_12::Zero();
    H.block<3, 3>(0, 3) = Matrix3_3::Identity();
    H.block<3, 3>(3, 6) = Matrix3_3::Identity();

    // Measurement Covariance Matrix
    double gps_pos_covar = 0.2;
    double gps_vel_covar = 0.7;
    Vector6 R_vec;
    R_vec << gps_pos_covar * gps_pos_covar * Vector3::Ones(), gps_vel_covar * gps_vel_covar * Vector3::Ones();
    Matrix6_6 R = R_vec.asDiagonal();

    // A priori covariance and Kalman gain
    Matrix12_6 L = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Predicted measurements
    Vector6 z_hat;
    z_hat << x_est.segment<3>(3), x_est.segment<3>(6);

    // Kalman Gain Weighting based on predicted acceleration
    Matrix12_12 ILH = (Matrix12_12::Identity() - L * H);
    P = ILH * P * ILH.transpose() + L * R * L.transpose();
    Vector6 residual = (z.segment<6>(9) - z_hat);
    Vector12 inn = L * residual;
    dx = dx + inn;
  }

  // Update full-state estimates
  q0 = sqrt(abs(1 - x_est.segment<3>(0).transpose() * x_est.segment<3>(0)));
  q << q0, x_est.segment<3>(0);
  Vector4 dq;
  dq << 1, dx.segment<3>(0) / 2;

  Vector4 q_nom = HamiltonianProd(q) * dq;
  q_nom.normalize();
  x_est.segment<3>(0) = q_nom.segment<3>(1).transpose();
  x_est.segment<9>(3) = x_est.segment<9>(3) + dx.segment<9>(3);
}
