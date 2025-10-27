function [x_est, dx] = EstimateStateFCN(x_est,constantsASTRA,z,dT,GND)

%% M-EKF Implementation
% Remove bias from gyro and accel, normalize mag
z(1:3) = z(1:3) - x_est(14:16);
z(4:6) = z(4:6) - x_est(11:13);
z(7:9) = z(7:9) / norm(z(7:9));

% Extract quaternion
dx = zeros(15,1);
q = x_est(1:4);

% Update the quaternion
M = HamiltonianProd(q);
qdot = 0.5 * M * [0; z(4:6)];
x_est(1:4) = x_est(1:4) + qdot * dT;
x_est(1:4) = x_est(1:4) / norm(x_est(1:4));

% A-priori quaternion rotation matrix
R_b2i = quatRot(q)';

% Process Covariance Matrix
persistent P lastZ iter
if isempty(P)
    P = 1 * eye(15);  
    lastZ = zeros(15,1);
    iter = 1;
end

% State Transition Matrix
F = StateTransitionMat(z(1:3), z(4:6), R_b2i);

% Propagate state using IMU
x_est(5:7) = x_est(5:7) + (R_b2i * z(1:3) - [0; 0; constantsASTRA.g]) * dT;
x_est(8:10) = x_est(8:10) + x_est(5:7) * dT;

% Discrete STM
Phi = expm(F * dT);

% Process Noise Covariance and a-priori propagation step
Q = 5 * constantsASTRA.Q;
R = constantsASTRA.R;
P = Phi * P * Phi' + Q;

if sum(lastZ(1:9) - z(1:9)) ~=0

    % Measurement matrix
    H = zeros(6,15);
    H(1:3, 1:3) = zetaCross(R_b2i' * [0; 0; constantsASTRA.g]);
    H(1:3, 13:15) = eye(3);
    H(4:6, 1:3) = zetaCross(R_b2i' * constantsASTRA.mag);

    % Measurement Noise Covariance
    w = 1 + 1e4 * (1 - GND);
    R(1:3,1:3) = R(1:3,1:3) * w;

    % A priori covariance and Kalman gain
    L = (P * H') / (H * P * H' + R);

    % Predicted measurements 
    z_hat = [R_b2i' * [0; 0; constantsASTRA.g];
             R_b2i' * constantsASTRA.mag];

    % Kalman Gain 
    ILH = (eye(15) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z([1:3 7:9]) - z_hat);
    dx = dx + L * residual;
end
if sum(lastZ(10:15) - z(10:15)) ~=0

    % Measurement matrix
    H = zeros(6,15);
    H(1:3, 7:9) = eye(3);
    H(4:6, 4:6) = eye(3);

    % Measurement Covariance Matrix
    gps_pos_covar = 2;
    gps_vel_covar = 2;
    R = diag([gps_pos_covar^2 * ones(3,1); gps_vel_covar^2 * ones(3,1)]);

    % A priori covariance and Kalman gain
    L = P * H' / (H * P * H' + R);

    % Predicted measurements 
    z_hat = [x_est(8:10);
             x_est(5:7)];

    % Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(15) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z(10:15) - z_hat);
    inn = L * residual;
    dx = dx + inn;
end

% Update full-state estimates
dq = [1; dx(1:3) / 2];

q_nom = HamiltonianProd(q) * dq;
q_nom = q_nom / norm(q_nom'); 
x_est(1:4) = q_nom;
x_est(5:16) = x_est(5:16) + dx(4:15);
lastZ = z;
iter = iter + 1;
end