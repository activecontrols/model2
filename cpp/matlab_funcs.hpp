#include <ArduinoEigenDense.h>

using Vector4 = Eigen::Matrix<float, 4, 1>;
using Vector10 = Eigen::Matrix<float, 10, 1>;
using Vector18 = Eigen::Matrix<float, 18, 1>;
using Matrix10_10 = Eigen::Matrix<float, 10, 10>;
using Matrix10_18 = Eigen::Matrix<float, 10, 18>;
using Matrix18_10 = Eigen::Matrix<float, 18, 10>;
using Matrix18_18 = Eigen::Matrix<float, 18, 18>;

Matrix18_18 AUG_JacobianX(Vector18 in1, Vector4 in2);
Vector18 AUG_plantfcn(Vector18 in1, Vector4 in2);
Vector18 EstimateState2(Vector10 Y, Vector18 X_hat, Vector4 U, float t, Matrix10_18 C);