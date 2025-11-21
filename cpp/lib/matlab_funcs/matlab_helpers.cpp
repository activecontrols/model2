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

Vector15 StateAUG(Vector13 XKF, Vector3 G) {
  // Change Filter State to Controls State
  Vector15 X;
  X << XKF.segment<3>(1), XKF.segment<3>(4), XKF.segment<3>(7), G - XKF.segment<3>(10), XKF.segment<3>(10);
  return X;
}

Vector12 ref_generator3(Vector15 full_x, Vector3 TargetPos) {
  Vector12 x = full_x.segment<12>(0);
  Vector3 PosError = TargetPos - x.segment<3>(3);
  Vector3 PosGain;
  PosGain << 0.55, 0.55, 0.75;
  Vector3 TargetVel = PosGain.cwiseProduct(PosError);

  float MaxAscentSpeed = 4;   // m/s
  float MaxDescentSpeed = -4; // m/s
  float MaxLatSpeed = 1;      // m/s

  Vector3 MaxVel;
  MaxVel << MaxLatSpeed, MaxLatSpeed, MaxAscentSpeed;
  Vector3 MinVel;
  MinVel << -MaxLatSpeed, -MaxLatSpeed, MaxDescentSpeed;

  TargetVel = TargetVel.cwiseMin(MaxVel).cwiseMax(MinVel);

  Vector12 TargetVec;
  TargetVec << Vector3::Zero(), TargetPos, TargetVel, Vector3::Zero();
  Vector12 ref = x - TargetVec;
  return ref;
}

Vector4 output_clamp(Vector4 U) {
  float thrust_max = 1.5 * 9.8; // N
  Vector4 maxU;
  maxU << M_PI / 24, M_PI / 24, thrust_max, thrust_max * 10;
  Vector4 minU;
  minU << -M_PI / 24, -M_PI / 24, thrust_max * 0.4, -thrust_max * 10;

  U = U.cwiseMin(maxU).cwiseMax(minU);
  return U;
}
