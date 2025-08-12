#include "matlab_funcs.hpp"

Matrix18_18 matrixExpPade6(Matrix18_18 A) { // TODO - does this code even work?
  using Scalar = typename Matrix18_18::Scalar;
  const Scalar b[] = {1.0, 0.5, 0.12, 0.0183333333333,
                      0.00199275362319, 0.000160590438368, 0.00000939085239292};

  Matrix18_18 A2 = A * A;
  Matrix18_18 A4 = A2 * A2;
  Matrix18_18 A6 = A4 * A2;

  Matrix18_18 U = A * (b[1] * Matrix18_18::Identity(A.rows(), A.cols()) +
                       b[3] * A2 + b[5] * A4);
  U += A6 * b[6];

  Matrix18_18 V = b[0] * Matrix18_18::Identity(A.rows(), A.cols()) +
                  b[2] * A2 + b[4] * A4 + b[6] * A6;

  Matrix18_18 numer = V + U;
  Matrix18_18 denom = V - U;

  return denom.inverse() * numer;
}

float rcond_est(Matrix10_10 A) { // TODO - does this code even work?
  using Scalar = typename Matrix10_10::Scalar;

  // Compute ||A||_1
  Scalar normA = 0;
  for (int j = 0; j < A.cols(); ++j) {
    Scalar colSum = 0;
    for (int i = 0; i < A.rows(); ++i)
      colSum += std::abs(A(i, j));
    normA = std::max(normA, colSum);
  }

  // Solve A * x = e_i repeatedly to estimate ||A^-1||_1
  Eigen::FullPivLU<Matrix10_10> lu(A);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> e = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(A.rows());
  Scalar maxColSumInv = 0;

  for (int i = 0; i < A.rows(); ++i) {
    e.setZero();
    e(i) = 1.0;
    auto x = lu.solve(e);
    Scalar colSum = 0;
    for (int k = 0; k < x.size(); ++k)
      colSum += std::abs(x(k));
    maxColSumInv = std::max(maxColSumInv, colSum);
  }

  return 1.0 / (normA * maxColSumInv);
}

Vector18 EstimateState2(Vector10 Y, Vector18 X_hat, Vector4 U, float t, Matrix10_18 C) {

  //// Single Kalman Filter Estimation (Extended Kalman Filter)
  Matrix18_18 P;

  //// Load System Dynamics
  // X_crit and U_crit are linearization points, for an EKF arquitechture we
  // linearize around the last estimated state and the current inputs.
  // Exception for T < 1 sec to avoid linearization around invalid points.

  // Calculate Jacobians
  Vector18 X_crit;
  Vector4 U_crit;

  if (t < 0.5) {
    X_crit = Vector18::Zero();
    U_crit << 0, 0, 1.5 * 9.8, 0; // fill top-to-bottom
  } else {
    X_crit = X_hat;
    U_crit = U;
  }

  // Jacobian function(specific to system dynamics, comprised of elementary // operations),
  // obtained thanks to MATLAB.
  Matrix18_18 J_x = AUG_JacobianX(X_crit, U_crit);
  // J_u = AUG_JacobianU(X_crit, U_crit); // not used currently

  // Covariance matrix for Kalman filter, initialized as an [n x n] matrix,
  // where n is the number of states (~13 for ASTRA v2)
  if (true) { // isempty (P) // TODO - setup this system
    P = Matrix18_18::Identity();
    X_hat << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.01, 0.12, 0.12, 0.12;
  }

  // Discretize the dynamics using zero order hold, standard operation.
  // timestep h to be chosen.
  float h = 1.0 / 500.0;
  Matrix18_18 A_d = matrixExpPade6(J_x * h); //.exp();

  // Measurements (Y is the measurements we get back, in this case it is
  // backed out by multiplying the observability matrix by the full state, but
  // since we're *not* supposed to know thw full state, we only use the
  // measurements. The observability matrix for ASTRA v2 depends on which
  // sensors we have, dimensions [m x n] where m is the number of
  // measurements. In our case, probably angular rates, positions (from GPS),
  // and accelerations.

  // Standard deviations of every state measurement (obtained experimentally
  // or just estimated)
  Vector18 Rvec;
  Rvec << 2, 2, 2, 2, 2, 2, 0.25, 0.25, 0.25, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;

  // Measurement Noise Covariancce Matrix
  Matrix10_10 R = (C * Rvec).array().square().matrix().asDiagonal();

  // Process Noise Covariance Matrix (obtained experimentally, size [n x n])
  Vector18 Q_tmp;
  Q_tmp << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-6;
  Matrix18_18 Q = Q_tmp.asDiagonal();

  Vector18 X_est;

  // Conditioning check for inverted matrix (look for the documentation of the
  // RCOND function in MATLAB for implementation)
  bool isWCond = rcond_est(C * P * C.transpose() + R * R.transpose()) > 1e-9;
  if (!isWCond) {
    // Set estimated state to -999 to signal ill-conditioned matrix and flag
    // the filter.
    X_est = Vector18::Ones() * -999;
  } else {
    // Calculates the Kalman Gain (inv is the matrix inverse.)
    Matrix18_10 L = A_d * P * C.transpose() * (C * P * C.transpose() + R * R.transpose()).inverse();

    // Prediction and Estimation (uses Euler integration to integrate the
    // state derivative obtained from the nonlinear plant dynamics).
    X_est = X_hat + AUG_plantfcn(X_hat, U) * h + L * (Y - C * X_hat);

    // Updates the covariance matrix.
    P = A_d * P * A_d.transpose() + Q * Q.transpose() - L * C * P * A_d.transpose();
  }

  return X_est;
}
