function [x_est, dx] = EstimateStateFCN(x_est,constantsASTRA,z,dT,GND)

%% M-EKF Implementation
% Filter mode (1 for full INS when GPS signals available, 0 for pure
% integration after launch when no GPS available | limit flight time)
FILTER_MODE = 1;

% Remove bias fromIMU
z(1:3) = z(1:3) - x_est(14:16) * (FILTER_MODE == 1 || GND == 1);
z(4:6) = z(4:6) - x_est(11:13) * (FILTER_MODE == 1 || GND == 1);
z(7:9) = z(7:9) - x_est(17:19) * (FILTER_MODE == 1 || GND == 1);

% Extract quaternion
dx = zeros(18,1);
q = x_est(1:4);
qdot = 0.5 * HamiltonianProd(q) * [0; z(4:6)]; 
x_est(1:4) = q + qdot * dT;
q = x_est(1:4);

% A-priori quaternion estimate and rotation matrix
q = q / norm(q);
R_b2i = quatRot(q)';

% Process Covariance Matrix
persistent P lastZ 
if isempty(P)
    P = 1 * eye(18);  
    lastZ = zeros(15,1);
end

% State Transition Matrix (CHANGE!!)
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
Q = 0.5 * Q;
P = Phi * P * Phi' + Q;
RTK = 0;

if sum(lastZ(1:9) - z(1:9)) ~=0 && (FILTER_MODE == 1 && GND == 1)

    % Measurement matrix
    H = zeros(6,18);
    H(1:3, 1:3) = zetaCross(R_b2i' * [0; 0; constantsASTRA.g]);
    H(1:3, 13:15) = eye(3);
    H(4:6, 1:3) = zetaCross(R_b2i' * constantsASTRA.mag);
    H(4:6, 16:18) = eye(3);

    % Measurement Noise Covariance
    w = 1 + 1e5 * (1 - GND);
    R(1:3,1:3) = R(1:3,1:3) * w;
    
    % A priori covariance and Kalman gain
    L = P * H' / (H * P * H' + R);
    
    % Predicted measurements 
    z_hat = [R_b2i' * [0; 0; constantsASTRA.g];
             R_b2i' * constantsASTRA.mag];
    
    % Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(18) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z([1:3 7:9]) - z_hat);
    dx = dx + L * residual;
end
if sum(lastZ(10:15) - z(10:15)) ~=0 && (FILTER_MODE == 1 || GND == 1)

    % Measurement matrix
    H = zeros(6,18);
    H(1:3, 4:6) = eye(3);
    H(4:6, 7:9) = eye(3);

    % Measurement Covariance Matrix
    gps_pos_covar = 1 * RTK + 150 * (1 - RTK);
    gps_vel_covar = gps_pos_covar * 0.1;
    R = diag([gps_pos_covar^2 * ones(3,1); gps_vel_covar^2 * ones(3,1)]);

    % A priori covariance and Kalman gain
    L = P * H' / (H * P * H' + R);

    % Predicted measurements 
    z_hat = [x_est(5:7);
             x_est(8:10)];

    % Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(18) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z(10:15) - z_hat);
    inn = L * residual;
    dx = dx + inn;
end
if FILTER_MODE == 1 || GND == 1
    % Update full-state estimates
    dq = [1; dx(1:3) / 2];
    dq = dq / norm(dq);
    
    q_nom = quatmultiply(q', dq');
    q_nom = q_nom / norm(q_nom); 
    dx(10:18) = dx(10:18) * GND;
    x_est(1:4) = q_nom';
    x_est(5:19) = x_est(5:19) + dx(4:18);
end
x_est(5:10) = x_est(5:10) * (1 - GND);
lastZ = z;
end