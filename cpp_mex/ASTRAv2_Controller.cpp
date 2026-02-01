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

Vector4 ASTRAv2_Controller(Vector3 PosTarget, Vector16 X, constantsASTRA_t constantsASTRA, float dT) {
  // Controller Limits
  float thrustMax = 1.5 * 9.8; // N
  float gimbalMax = M_PI / 18;
  Vector4 uMin = (Vector4() << -gimbalMax, -gimbalMax, 0.4 * thrustMax, -M_PI / 6).finished();
  Vector4 uMax = (Vector4() << gimbalMax, gimbalMax, thrustMax, M_PI / 6).finished();
  Vector4 U = Vector4::Zero();

  // First Loop (P Loop)
  // Position Error Vector
  Vector3 PosError = PosTarget - X.segment<3>(4);

  // Velocity Command
  Vector3 K_P = (Vector3() << 0.7, 0.7, 0.65).finished();
  Vector3 VelTarget = K_P.cwiseProduct(PosError);

  // Velocity Saturation Step
  Vector3 MaxVel = (Vector3() << 1, 1, 1.5).finished();
  VelTarget = VelTarget.cwiseMin(MaxVel).cwiseMax(-MaxVel);

  // Second Loop (PI Loop)
  // Velocity Error Vector
  Vector3 VelError = VelTarget - X.segment<3>(7);

  // Integral Accumulator
  Vector3 K_I = (Vector3() << 2, 2, 5).finished();
  float Leak = 0.35;
  Vector3 Clamp = (Vector3() << 5, 5, 5).finished();

  // Normalize errors (0 to 1 scale)
  Vector3 MaxAttError = (Vector3() << 0.06, 0.06, 0.3).finished();
  Vector3 MaxVelError = (Vector3() << 0.3, 0.3, 0.6).finished();
  Vector3 NormAttErr = lastAttError.cwiseAbs().cwiseQuotient(MaxAttError);
  Vector3 NormVelErr = VelError.cwiseAbs().cwiseQuotient(MaxVelError);

  // Combine errors (Vector magnitude)
  Vector3 TotalErrorMetric = NormAttErr + NormVelErr;

  // Calculate Gate using Gaussian function
  Vector3 Gate = (1 - Leak) * (-2 * TotalErrorMetric.cwiseAbs2()).array().exp().matrix() + (Vector3() << Leak, Leak, Leak).finished();

  // Integrator
  K_I = K_I.cwiseProduct(Gate);
  VelErrorI = VelErrorI + K_I.cwiseProduct(VelError) * dT;
  VelErrorI = VelErrorI.cwiseMin(Clamp).cwiseMax(-Clamp);
  K_P = (Vector3() << 2.4, 2.4, 5).finished();

  // Acceleration Target
  Vector3 AccelTarget = K_P.cwiseProduct(VelError) + VelErrorI + (Vector3() << 0, 0, constantsASTRA.g).finished();

  // Acceleration Saturation Step
  Vector3 MaxAccelUp = (Vector3() << 2, 2, 15).finished();
  Vector3 MaxAccelDown = (Vector3() << -2, -2, 4).finished();
  AccelTarget = AccelTarget.cwiseMin(MaxAccelUp).cwiseMax(MaxAccelDown);

  // Kinematics Step
  // Compute thrust target
  Vector3 TargetForce_I = constantsASTRA.m * AccelTarget;
  U[2] = TargetForce_I.norm();

  // Compute target attitude via GSP.
  AccelTarget[2] = max(AccelTarget[2], constantsASTRA.g);
  Vector3 Z_b = AccelTarget / AccelTarget.norm();

  // Heading reference (+X axis rolled to north)
  Vector3 HDGRef = (Vector3() << 1, 0, 0).finished();
  Vector3 Y_b = Z_b.cross(HDGRef);
  Y_b.normalize();

  // Complete the triad
  Vector3 X_b = Y_b.cross(Z_b);

  // Create DCM and convert to quaternion
  Matrix3_3 DCM;
  DCM << X_b[0], Y_b[0], Z_b[0], X_b[1], Y_b[1], Z_b[1], X_b[2], Y_b[2], Z_b[2];
  Vector4 TargetAtt = DCM_Quat_Conversion(DCM);

  // Third Loop (LQRi)
  // Attitude Error computation
  Vector4 Q_Conj = (Vector4() << X[0], -X.segment<3>(1)).finished();
  Vector4 AttError = HamiltonianProd(Q_Conj) * TargetAtt;
  if (AttError[0] < 0) {
    AttError = -AttError;
  }
  lastAttError = AttError.segment<3>(1);

  // Error accumulation and clamping
  Clamp = (Vector3() << 3, 3, 0.4).finished();
  AttErrorI = AttErrorI + AttError.segment<3>(1) * dT;
  AttErrorI = AttErrorI.cwiseMin(Clamp).cwiseMax(-Clamp);

  // State vector and error
  Vector9 X_Err = (Vector9() << -AttError.segment<3>(1), X.segment<3>(10), AttErrorI).finished();

  // LQR Controller
  Vector3 u_components = -constantsASTRA.K_Att * X_Err;
  U[0] = u_components[0];
  U[1] = u_components[1];
  U[3] = u_components[2];

  // Controls Saturation
  U = U.cwiseMax(uMin).cwiseMin(uMax);
  return U;
}
