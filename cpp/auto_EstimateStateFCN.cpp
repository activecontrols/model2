Vector12   EstimateStateFCN(Vector12 x_est, struct constantsASTRA, Vector15 z, Vector3 covar_vec, double dT, Matrix12_12 Q, double GND) {

//// M-EKF Implementation
// Propagate nominal state (DEAD-RECKONING PROPAGATION FOR Q, R, AND V)
// Extract Quaternion
// REMOVE RATES FROM STATE_VEC, AIM FOR FULL STATE SIM

// Remove bias from gyro
z(4:6) = z(4:6) - x_est(10:12);

// Extract quaternion
Vector12 dx = zeros(12,1);
double q0 = sqrt(abs(1 - x_est(1:3).transpose()*x_est(1:3)));
Vector4 q = [q0; x_est(1:3)];
Matrix4_4 M = [q(1) -q(2) -q(3) -q(4);
     q(2)  q(1) -q(4)  q(3);
     q(3)  q(4)  q(1) -q(2);
     q(4) -q(3)  q(2)  q(1)];
Vector4 qdot = 0.5 * M * [0; z(4:6)];
Vector3 q_123_dot = qdot(2:4);

x_dot = nominalDynamics([x_est(1:9); z(4:6); x_est(10:12)], u);
x_est(1:3) = q(2:4) + q_123_dot * dT;

// A-priori quaternion estimate and rotation matrix
q0 = sqrt(1 - x_est(1:3).transpose()*x_est(1:3));
q = [q0; x_est(1:3)];
Matrix3_3 R_b2i = quatRot(q).transpose();

// Process Covariance Matrix
persistent P lastZ
if isempty(P) {
Matrix12_12     P = 1 * eye(12);
    lastZ = zeros(15,1);
}

// State Transition Matrix
Matrix12_12 F = StateTransitionMat(z(1:3), z(4:6), R_b2i);

// Propagate rest of state using IMU
x_est(7:9) = x_est(7:9) + (R_b2i * z(1:3) - [0; 0; constantsASTRA.g]) * dT;
x_est(4:6) = x_est(4:6) + x_est(7:9) * dT;

// Discrete STM
Matrix12_12 Phi = expm(F * dT);

// Process Noise Covariance and a-priori propagation step
Matrix12_12 Q = 0.4 * Q;
P = Phi * P * Phi.transpose() + Q;

if sum(lastZ(1:9) - z(1:9)) ~=0 {

    // Measurement matrix
Matrix6_12     H = zeros(6,12);
    H(1:3, 1:3) = zetaCross(R_b2i.transpose() * [0; 0; constantsASTRA.g]);
    H(4:6, 1:3) = zetaCross(R_b2i.transpose() * constantsASTRA.mag);

    // Measurement Noise Covariance
double     w = 1 + 300 * (1 - GND);
Matrix6_6     R = diag([(covar_vec(1) * w)^2 * ones(3,1); covar_vec(3)^2 * ones(3,1)]);

    // A priori covariance and Kalman gain
Matrix12_6     L = P * H.transpose() / (H * P * H.transpose() + R);

    // Predicted measurements
Vector6     z_hat = [R_b2i.transpose() * [0; 0; constantsASTRA.g];
             R_b2i.transpose() * constantsASTRA.mag];

    // Kalman Gain Weighting based on predicted acceleration
Matrix12_12     ILH = (eye(12) - L * H);
    P = ILH * P * ILH.transpose() + L * R * L.transpose();
Vector6     residual = (z([1:3 7:9]) - z_hat);
    dx = dx + L * residual;
}
if sum(lastZ(10:15) - z(10:15)) ~=0 {

    // Measurement matrix
    H = zeros(6,12);
    H(1:3, 4:6) = eye(3);
    H(4:6, 7:9) = eye(3);

    // Measurement Covariance Matrix
    gps_pos_covar = 0.2;
    gps_vel_covar = 0.7;
    R = diag([gps_pos_covar^2 * ones(3,1); gps_vel_covar^2 * ones(3,1)]);

    // A priori covariance and Kalman gain
    L = P * H.transpose() / (H * P * H.transpose() + R);

    // Predicted measurements
    z_hat = [x_est(4:6);
             x_est(7:9)];

    // Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(12) - L * H);
    P = ILH * P * ILH.transpose() + L * R * L.transpose();
    residual = (z(10:15) - z_hat);
Vector12     inn = L * residual;
    dx = dx + inn;
}

// Update full-state estimates
q0 = sqrt(abs(1 - x_est(1:3).transpose()*x_est(1:3)));
q = [q0; x_est(1:3)];
Vector4 dq = [1; dx(1:3) / 2];

Vector4 q_nom = quatmultiply(q.transpose(), dq.transpose());
q_nom = q_nom / norm(q_nom);
x_est(1:3) = q_nom(2:4).transpose();
x_est(4:12) = x_est(4:12) + dx(4:12);
lastZ = z;
}
