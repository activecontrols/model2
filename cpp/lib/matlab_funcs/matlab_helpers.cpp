#include "matlab_funcs.hpp"

Matrix3_3 zetaCross(Vector3 zeta) {
  Matrix3_3 zc;
  zc << 0, -zeta[2], zeta[1], zeta[2], 0, -zeta[0], -zeta[1], zeta[0], 0;
  return zc;
}

Matrix3_3 quatRot(Vector4 q) {
  Matrix3_3 CIB;
  CIB << 1 - 2 * (q[2] * q[2] + q[3] * q[3]), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2]),
      2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] * q[1] + q[3] * q[3]), 2 * (q[2] * q[3] + q[0] * q[1]),
      2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  return CIB;
}

Matrix4_4 HamiltonianProd(Vector4 q) {
  Matrix4_4 M;
  M << q(0), -q(1), -q(2), -q(3),
      q(1), q(0), -q(3), q(2),
      q(2), q(3), q(0), -q(1),
      q(3), -q(2), q(1), q(0);
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

Matrix12_12 matrixExpPade6(Matrix12_12 A) { // TODO - does this code even work?
  using Scalar = typename Matrix12_12::Scalar;
  const Scalar b[] = {1.0, 0.5, 0.12, 0.0183333333333,
                      0.00199275362319, 0.000160590438368, 0.00000939085239292};

  Matrix12_12 A2 = A * A;
  Matrix12_12 A4 = A2 * A2;
  Matrix12_12 A6 = A4 * A2;

  Matrix12_12 U = A * (b[1] * Matrix12_12::Identity(A.rows(), A.cols()) +
                       b[3] * A2 + b[5] * A4);
  U += A6 * b[6];

  Matrix12_12 V = b[0] * Matrix12_12::Identity(A.rows(), A.cols()) +
                  b[2] * A2 + b[4] * A4 + b[6] * A6;

  Matrix12_12 numer = V + U;
  Matrix12_12 denom = V - U;

  return (denom.inverse() * numer).transpose(); // NOTE RJN - had to add this transpose - don't know why
}
