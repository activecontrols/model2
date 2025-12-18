#pragma once
#include <ArduinoEigenDense.h>

// Note - using matlab funcs based off of ASTRA simulation: e5e0557ebb837f220d9740cd96e1020724bceb48
// TODO - update matlab func commit

using Vector3 = Eigen::Matrix<float, 3, 1>;
using Vector4 = Eigen::Matrix<float, 4, 1>;
using Vector6 = Eigen::Matrix<float, 6, 1>;
using Vector9 = Eigen::Matrix<float, 9, 1>;
using Vector12 = Eigen::Matrix<float, 12, 1>;
using Vector13 = Eigen::Matrix<float, 13, 1>;
using Vector15 = Eigen::Matrix<float, 15, 1>;
using Vector16 = Eigen::Matrix<float, 16, 1>;
using Matrix2_2 = Eigen::Matrix<float, 2, 2>;
using Matrix3_3 = Eigen::Matrix<float, 3, 3>;
using Matrix3_9 = Eigen::Matrix<float, 3, 9>;
using Matrix4_4 = Eigen::Matrix<float, 4, 4>;
using Matrix6_6 = Eigen::Matrix<float, 6, 6>;
using Matrix6_12 = Eigen::Matrix<float, 6, 12>;
using Matrix9_4 = Eigen::Matrix<float, 9, 4>;
using Matrix12_6 = Eigen::Matrix<float, 12, 6>;
using Matrix12_12 = Eigen::Matrix<float, 12, 12>;

typedef struct {
  float g;
  float m;
  Vector3 mag;
  Matrix12_12 Q;
  Matrix6_6 R;
  Matrix3_9 K_Att;
} t_constantsASTRA;

Matrix3_3 zetaCross(Vector3 zeta);
Matrix3_3 quatRot(Vector4 q);
Matrix12_12 StateTransitionMat(Vector3 accel, Vector3 gyro, Matrix3_3 R_b2i);
Matrix4_4 HamiltonianProd(Vector4 q);
Matrix12_12 matrixExpPade6(Matrix12_12 A);
Vector13 EstimateStateFCN(Vector13 x_est, t_constantsASTRA constantsASTRA, Vector15 z, float dT, float GND, Matrix12_12 &P, bool new_imu_packet, bool new_gps_packet);
Vector3 EMA_Gyros(Vector15 Y, Vector3 &lastEMA);
Vector16 StateAUG(Vector13 XKF, Vector3 G);
Vector4 DCM_Quat_Conversion(Matrix3_3 R);
Vector9 DigitalNF(Vector9 IN, float GND, float THRUST, float dT, Matrix9_4 &X, Matrix9_4 &Y);
void ASTRAv2_Controller_reset();
Vector4 ASTRAv2_Controller(Vector3 PosTarget, Vector16 X, t_constantsASTRA constantsASTRA, float dT);
