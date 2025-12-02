#include "matlab_funcs.h"

// Version 2 Controller formulation for ASTRAv2. Structure consists of 3
// cascaded loops.
//
// First Loop is a P Loop in charge of Pos -> Vel commands.
//
// Second loop is a PI Loop in charge of Vel -> Acceleration commands, fitted
// with anti-windup and soft-gatingcbased on attitude error.
//
// Second loop is an LQRi Loop in charge of Target Attitude -> Gimbal +
// Torque commands. Integral action is fitted with anti-windup clamps and is
// self soft-gated.
// Tuning of the second loop is done by using a Genetic algorithm to
// maximize a Crossover Frequency vs. Disk Margin tradeoff on all actuator
// channels.
//
// By: Pablo Plata   -   11/27/25 (Happy Thanksgiving!)

Vector3 VelErrorI;
Vector3 AttErrorI;
Vector3 lastAttError;

void ASTRAv2_Controller_reset() {
  VelErrorI = Vector3::Zero();
  AttErrorI = Vector3::Zero();
  lastAttError = Vector3::Zero();
}

Vector4 ASTRAv2_Controller(Vector3 PosTarget, Vector16 X, t_constantsASTRA constantsASTRA, float dT) {
  // Controller Limits
  float thrustMax = 1.5 * 9.8; // N
  float gimbalMax = M_PI / 18;

  Vector4 uMin;
  uMin << -gimbalMax, -gimbalMax, 0.4 * thrustMax, -M_PI / 6;
  Vector4 uMax;
  uMax << gimbalMax, gimbalMax, thrustMax, M_PI / 6;

  Vector4 U = Vector4::Zero();

  //// First Loop (P Loop)
  // Position Error Vector
  Vector3 PosError = PosTarget - X.segment<3>(4);

  // Velocity Command
  Vector3 K_P;
  K_P << 0.5, 0.5, 0.65;
  Vector3 VelTarget = K_P.cwiseProduct(PosError);

  // Velocity Saturation Step
  Vector3 MaxVel;
  MaxVel << 1, 1, 1.5;
  VelTarget = VelTarget.cwiseMin(MaxVel).cwiseMax(-MaxVel);

  //// Second Loop (PI Loop)
  // Velocity Error Vector
  Vector3 VelError = VelTarget - X.segment<3>(7);

  // Integral Accumulator
  Vector3 K_I;
  K_I << 1.5, 1.5, 5;
  float Leak = 0.10;
  Vector3 Clamp;
  Clamp << 1, 1, 2;

  // Normalize errors (0 to 1 scale)
  Vector3 MaxAttError;
  MaxAttError << 0.07, 0.07, 0.3;
  Vector3 MaxVelError;
  MaxVelError << 0.6, 0.6, 0.4;
  Vector3 MaxRateError;
  MaxRateError << M_PI / 5, M_PI / 5, M_PI / 3;
  Vector3 NormAttErr = lastAttError.cwiseAbs().cwiseQuotient(MaxAttError);
  Vector3 NormVelErr = VelError.cwiseAbs().cwiseQuotient(MaxVelError);
  Vector3 NormRateErr = X.segment<3>(10).cwiseAbs().cwiseQuotient(MaxRateError);

  // Combine errors (Vector magnitude)
  Vector3 TotalErrorMetric = NormAttErr + NormVelErr + NormRateErr;

  // Calculate Gate using Gaussian function
  Vector3 LeakVec3;
  LeakVec3 << Leak, Leak, Leak;
  Vector3 Gate = (1 - Leak) * (-2 * TotalErrorMetric.cwiseAbs2()).array().exp().matrix() + LeakVec3;

  // Integrator
  K_I = K_I.cwiseProduct(Gate);
  VelErrorI = VelErrorI + K_I.cwiseProduct(VelError) * dT;
  VelErrorI = VelErrorI.cwiseMin(Clamp).cwiseMax(-Clamp);
  K_P << 2.2, 2.2, 3.5;

  // Acceleration Target
  Vector3 g_vec;
  g_vec << 0, 0, constantsASTRA.g;
  Vector3 AccelTarget = K_P.cwiseProduct(VelError) + VelErrorI + g_vec;

  // Acceleration Saturation Step
  Vector3 MaxAccelUp;
  MaxAccelUp << 2, 2, 15;
  Vector3 MaxAccelDown;
  MaxAccelDown << -2, -2, 4;
  AccelTarget = AccelTarget.cwiseMin(MaxAccelUp).cwiseMax(MaxAccelDown);

  //// Kinematics Step
  // Compute thrust target
  Vector3 TargetForce_I = constantsASTRA.m * AccelTarget;
  U[2] = TargetForce_I.norm();

  // Compute target attitude via GSP.
  AccelTarget[2] = max(AccelTarget[2], constantsASTRA.g);
  Vector3 Z_b = AccelTarget / AccelTarget.norm();

  // Heading reference (+X axis rolled to north)
  Vector3 HDGRef;
  HDGRef << 1, 0, 0;
  Vector3 Y_b = Z_b.cross(HDGRef);
  Y_b = Y_b / Y_b.norm();

  // Complete the triad
  Vector3 X_b = Y_b.cross(Z_b);

  // Create DCM and convert to quaternion
  Matrix3_3 DCM;
  DCM << X_b[0], Y_b[0], Z_b[0], X_b[1], Y_b[1], Z_b[1], X_b[2], Y_b[2], Z_b[2];
  Vector4 TargetAtt = DCM_Quat_Conversion(DCM);

  //// Third Loop (LQRi)
  // Attitude Error computation
  Vector4 Q_Conj;
  Q_Conj << X[0], -X.segment<3>(1);
  Vector4 AttError = HamiltonianProd(Q_Conj) * TargetAtt;
  if (AttError[0] < 0) {
    AttError = -AttError;
  }
  lastAttError = AttError.segment<3>(1);

  // Error accumulation and clamping
  Clamp << 3, 3, 0.4;
  AttErrorI = AttErrorI + AttError.segment<3>(1) * dT;
  AttErrorI = AttErrorI.cwiseMin(Clamp).cwiseMax(-Clamp);

  // State vector and error
  Vector9 X_Err;
  X_Err << -AttError.segment<3>(1), X.segment<3>(10), AttErrorI;

  // LQR Controller
  Vector3 u_components = -constantsASTRA.K_Att * X_Err;
  U[0] = u_components[0];
  U[1] = u_components[1];
  // U[2] is set above
  U[3] = u_components[2];

  //// Controls Saturation
  U = U.cwiseMin(uMax).cwiseMax(uMin);

  return U;
}
