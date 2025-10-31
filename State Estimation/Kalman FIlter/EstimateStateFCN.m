function [x_est, dx] = EstimateStateFCN(x_est,constantsASTRA,z,dT,GND)

%% M-EKF Implementation
% Remove bias from gyro
z(4:6) = z(4:6) - x_est(11:13);

% Extract quaternion
dx = zeros(12,1);
%q0 = sqrt(abs(1 - x_est(1:3)'*x_est(1:3)));
%q = [q0; x_est(1:3)];
q = x_est(1:4);
qdot = 0.5 * HamiltonianProd(q) * [0; z(4:6)];
%q_123_dot = qdot(2:4); 
x_est(1:4) = q + qdot * dT;
q = x_est(1:4);

% A-priori quaternion estimate and rotation matrix
%q0 = sqrt(1 - x_est(1:3)'*x_est(1:3));
%q = [q0; x_est(1:3)];
q = q / norm(q);
R_b2i = quatRot(q)';

% Process Covariance Matrix
persistent P lastZ 
if isempty(P)
    P = 1 * eye(12);  
    lastZ = zeros(15,1);
end

% State Transition Matrix
F = StateTransitionMat(z(1:3), z(4:6), R_b2i);

% Propagate rest of state using IMU
x_est(8:10) = x_est(8:10) + (R_b2i * z(1:3) - [0; 0; constantsASTRA.g]) * dT;
x_est(5:7) = x_est(5:7) + x_est(8:10) * dT;

% Discrete STM
Phi = expm(F * dT);

% Extract Matrices
Q = constantsASTRA.Q;
R = constantsASTRA.R;

% Process Noise Covariance and a-priori propagation step
Q = 0.4 * Q;
P = Phi * P * Phi' + Q;

if sum(lastZ(1:9) - z(1:9)) ~=0

    % Measurement matrix
    H = zeros(6,12);
    H(1:3, 1:3) = zetaCross(R_b2i' * [0; 0; constantsASTRA.g]);
    H(4:6, 1:3) = zetaCross(R_b2i' * constantsASTRA.mag);

    % Measurement Noise Covariance
    w = 1 + 1e5 * (1 - GND);
    R(1:3,1:3) = R(1:3,1:3) * w;
    
    % A priori covariance and Kalman gain
    L = P * H' / (H * P * H' + R);
    
    % Predicted measurements 
    z_hat = [R_b2i' * [0; 0; constantsASTRA.g];
             R_b2i' * constantsASTRA.mag];
    
    % Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(12) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z([1:3 7:9]) - z_hat);
    dx = dx + L * residual;
end
if sum(lastZ(10:15) - z(10:15)) ~=0

    % Measurement matrix
    H = zeros(6,12);
    H(1:3, 4:6) = eye(3);
    H(4:6, 7:9) = eye(3);

    % Measurement Covariance Matrix
    gps_pos_covar = 0.2;
    gps_vel_covar = 1.25;
    R = diag([gps_pos_covar^2 * ones(3,1); gps_vel_covar^2 * ones(3,1)]);

    % A priori covariance and Kalman gain
    L = P * H' / (H * P * H' + R);

    % Predicted measurements 
    z_hat = [x_est(5:7);
             x_est(8:10)];

    % Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(12) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z(10:15) - z_hat);
    inn = L * residual;
    dx = dx + inn;
end

% Update full-state estimates
% q0 = sqrt(abs(1 - x_est(1:3)'*x_est(1:3)));
% q = [q0; x_est(1:3)];
dq = [1; dx(1:3) / 2];
dq = dq / norm(dq);

q_nom = quatmultiply(q', dq');
q_nom = q_nom / norm(q_nom); 
x_est(1:4) = q_nom';
x_est(5:13) = x_est(5:13) + dx(4:12);
lastZ = z;
end