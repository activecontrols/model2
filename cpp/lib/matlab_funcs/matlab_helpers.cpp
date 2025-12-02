#include "matlab_funcs.h"

Matrix3_3 zetaCross(Vector3 zeta) {
  Matrix3_3 zc;
  zc << 0, -zeta[2], zeta[1], zeta[2], 0, -zeta[0], -zeta[1], zeta[0], 0;
  return zc;
}

Matrix3_3 quatRot(Vector4 q) {
  Matrix3_3 CIB;
  CIB << 1 - 2 * (q[2] * q[2] + q[3] * q[3]), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] * q[1] + q[3] * q[3]),
      2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  return CIB;
}

Matrix4_4 HamiltonianProd(Vector4 q) {
  Matrix4_4 M;
  M << q(0), -q(1), -q(2), -q(3), q(1), q(0), -q(3), q(2), q(2), q(3), q(0), -q(1), q(3), -q(2), q(1), q(0);
  return M;
}

Matrix12_12 StateTransitionMat(Vector3 accel, Vector3 gyro, Matrix3_3 R_b2i) {
  // Remove angular rates from error-state. Make safety copies of all relevant
  // files into Archive
  Matrix12_12 F = Matrix12_12::Zero();
  F.block<3, 3>(0, 0) = -zetaCross(gyro);
  F.block<3, 3>(0, 9) = -Matrix3_3::Identity();
  F.block<3, 3>(3, 6) = Matrix3_3::Identity();
  F.block<3, 3>(6, 0) = -R_b2i * zetaCross(accel);

  return F;
}

Matrix12_12 matrixExpPade6(Matrix12_12 A) {
  using Scalar = typename Matrix12_12::Scalar;
  const Scalar b[] = {1.0, 0.5, 0.12, 0.0183333333333, 0.00199275362319, 0.000160590438368, 0.00000939085239292};

  Matrix12_12 A2 = A * A;
  Matrix12_12 A4 = A2 * A2;
  Matrix12_12 A6 = A4 * A2;

  Matrix12_12 U = A * (b[1] * Matrix12_12::Identity(A.rows(), A.cols()) + b[3] * A2 + b[5] * A4);
  U += A6 * b[6];

  Matrix12_12 V = b[0] * Matrix12_12::Identity(A.rows(), A.cols()) + b[2] * A2 + b[4] * A4 + b[6] * A6;

  Matrix12_12 numer = V + U;
  Matrix12_12 denom = V - U;

  return denom.inverse() * numer;
}

// Basic Exponential Moving Average Implementation to pre - process % measurements.
Vector3 ExpMovingAvg(Vector3 Input, Vector3 Last, float Alpha) {
  // Filter difference equation
  return Alpha * Input + (1 - Alpha) * Last;
}

Vector3 EMA_Gyros(Vector15 Y, Vector3 &lastEMA) {

  // Extract Gyros
  Vector3 gyros = Y.segment<3>(3);

  // Exponential Moving Avg Step
  Vector3 EMA_G = ExpMovingAvg(gyros, lastEMA, 0.3);
  lastEMA = EMA_G;
  return EMA_G;
}

Vector16 StateAUG(Vector13 XKF, Vector3 G) {
  // Change Filter State to Controls State
  Vector16 X;
  X << XKF.segment<4>(0), XKF.segment<3>(4), XKF.segment<3>(7), G - XKF.segment<3>(10), XKF.segment<3>(10);
  return X;
}

// Convert a DCM to a quaternion robustly
Vector4 DCM_Quat_Conversion(Matrix3_3 R) {

  // Extract trace
  float tr = R(0, 0) + R(1, 1) + R(2, 2);

  // Pre-allocate output
  Vector4 q = Vector4::Zero();

  // CASE 1: Standard Case (Trace > 0)
  // This works for most orientations (small angles, < 90 deg tilt)
  if (tr > 0) {
    float S = sqrt(tr + 1.0) * 2; // S = 4 * qw
    q[0] = 0.25 * S;
    q[1] = (R(2, 1) - R(1, 2)) / S;
    q[2] = (R(0, 2) - R(2, 0)) / S;
    q[3] = (R(1, 0) - R(0, 1)) / S;
  }
  // CASE 2: Singularity Avoidance (Trace <= 0)
  // We must find the largest diagonal element to avoid dividing by zero.
  else {
    if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
      // Column 1 (X) is dominant
      float S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2;
      q[0] = (R(2, 1) - R(1, 2)) / S;
      q[1] = 0.25 * S;
      q[2] = (R(0, 1) + R(1, 0)) / S;
      q[3] = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
      // Column 2 (Y) is dominant
      float S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;
      q[0] = (R(0, 2) - R(2, 0)) / S;
      q[1] = (R(0, 1) + R(1, 0)) / S;
      q[2] = 0.25 * S;
      q[3] = (R(1, 2) + R(2, 1)) / S;
    } else {
      // Column 3 (Z) is dominant
      float S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2;
      q[0] = (R(1, 0) - R(0, 1)) / S;
      q[1] = (R(0, 2) + R(2, 0)) / S;
      q[2] = (R(1, 2) + R(2, 1)) / S;
      q[3] = 0.25 * S;
    }
  }

  // Enforce normalization to correct numerical drift
  q.normalize();

  return q;
}