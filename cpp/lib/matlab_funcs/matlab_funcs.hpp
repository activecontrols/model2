#include <ArduinoEigenDense.h>

using Vector3 = Eigen::Matrix<float, 3, 1>;
using Vector4 = Eigen::Matrix<float, 4, 1>;
using Vector6 = Eigen::Matrix<float, 6, 1>;
using Vector12 = Eigen::Matrix<float, 12, 1>;
using Vector13 = Eigen::Matrix<float, 13, 1>;
using Vector15 = Eigen::Matrix<float, 15, 1>;
using Matrix3_3 = Eigen::Matrix<float, 3, 3>;
using Matrix4_4 = Eigen::Matrix<float, 4, 4>;
using Matrix6_6 = Eigen::Matrix<float, 6, 6>;
using Matrix6_12 = Eigen::Matrix<float, 6, 12>;
using Matrix12_6 = Eigen::Matrix<float, 12, 6>;
using Matrix12_12 = Eigen::Matrix<float, 12, 12>;

typedef struct {
  float g;
  Vector3 mag;
  Matrix12_12 Q;
  Matrix6_6 R;
} t_constantsASTRA;

Matrix3_3 zetaCross(Vector3 zeta);
Matrix3_3 quatRot(Vector4 q);
Matrix12_12 StateTransitionMat(Vector3 accel, Vector3 gyro, Matrix3_3 R_b2i);
Matrix4_4 HamiltonianProd(Vector4 q);
Matrix12_12 matrixExpPade6(Matrix12_12 A);
Vector13 EstimateStateFCN(Vector13 x_est, t_constantsASTRA constantsASTRA, Vector15 z, float dT, float GND, Matrix12_12 &P, bool new_imu_packet, bool new_gps_packet);