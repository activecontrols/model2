#include <ArduinoEigenDense.h>

using Vector3 = Eigen::Matrix<double, 3, 1>;
using Vector4 = Eigen::Matrix<double, 4, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector12 = Eigen::Matrix<double, 12, 1>;
using Vector15 = Eigen::Matrix<double, 15, 1>;
using Matrix3_3 = Eigen::Matrix<double, 3, 3>;
using Matrix4_4 = Eigen::Matrix<double, 4, 4>;
using Matrix6_6 = Eigen::Matrix<double, 6, 6>;
using Matrix6_12 = Eigen::Matrix<double, 6, 12>;
using Matrix12_6 = Eigen::Matrix<double, 12, 6>;
using Matrix12_12 = Eigen::Matrix<double, 12, 12>;

typedef struct {
  double m;
  double l;
  double g;
  double rTB;
  Matrix3_3 j;
  double T;
  Vector3 mag;
} t_constantsASTRA;

Matrix3_3 zetaCross(Vector3 zeta);
Matrix3_3 quatRot(Vector4 q);
Matrix12_12 StateTransitionMat(Vector3 accel, Vector3 gyro, Matrix3_3 R_b2i);
Matrix4_4 HamiltonianProd(Vector4 q);
Matrix12_12 matrixExpPade6(Matrix12_12 A);
Vector12 EstimateStateFCN(Vector12 x_est, t_constantsASTRA constantsASTRA, Vector15 z, Vector3 covar_vec, double dT, Matrix12_12 Q, double GND, Matrix12_12 P, bool new_imu_packet, bool new_gps_packet);