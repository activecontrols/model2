function [x_est, dx] = EstimateStateFCN(x_est,u,constantsASTRA,z,covar_vec,dT,t,Q)

%% M-EKF Implementation
% Propagate nominal state (DEAD-RECKONING PROPAGATION FOR Q, R, AND V)
% Extract Quaternion
% ISSUES WITH QUATERNION CONVERGENCE - TRACK COVARIANCES AND FIX Q_k MATRIX

% Remove bias from gyro
z(4:6) = z(4:6) - x_est(13:15);

% Extract quaternion
dx = zeros(15,1);
q0 = sqrt(abs(1 - x_est(1:3)'*x_est(1:3)));
q = [q0; x_est(1:3)];
M = [q(1) -q(2) -q(3) -q(4);
     q(2)  q(1) -q(4)  q(3);
     q(3)  q(4)  q(1) -q(2);
     q(4) -q(3)  q(2)  q(1)];
qdot = 0.5 * M * [0; z(4:6)]; %body frame derivative
q_123_dot = qdot(2:4); % + zetaCross(z(4:6)) * q(2:4); %inertial derivative 
x_est(1:3) = q(2:4) + q_123_dot * dT;

% A-priori quaternion estimate and rotation matrix
q0 = sqrt(1 - x_est(1:3)'*x_est(1:3));
q = [q0; x_est(1:3)];
R_b2i = quatRot(q)';

% Process Covariance Matrix
persistent P lastZ 
if isempty(P)
    P = 0.2 * eye(15);  
    lastZ = zeros(15,1);
end

% State Transition Matrix
F = StateTransitionMat(z(1:3), z(4:6), R_b2i, constantsASTRA.J);

% Propagate rest of state
x_dot = nominalDynamics(x_est, u);
x_est(7:9) = x_est(7:9) + (R_b2i * z(1:3) - [0; 0; constantsASTRA.g]) * dT;
x_est(4:6) = x_est(4:6) + x_est(7:9) * dT;
x_est(10:12) = x_est(10:12) + x_dot(10:12) * dT;

% Discrete STM
Phi = expm(F * dT);

% Process Noise Covariance and a-priori propagation step
Q = 0.5*Q;
P = Phi * P * Phi' + Q;

if sum(lastZ(1:9) - z(1:9)) ~=0

    % Measurement matrix
    H = zeros(9,15);
    H(1:3, 1:3) = zetaCross(R_b2i' * [0; 0; constantsASTRA.g]);
    H(4:6, 10:12) = eye(3);
    H(4:6, 13:15) = eye(3);
    H(7:9, 1:3) = zetaCross(R_b2i' * constantsASTRA.mag);

    % Measurement Noise Covariance
    w = 1 + 20 * x_dot(7:9)' * x_dot(7:9) + 5 * x_dot(4:6)' * x_dot(4:6);
    w = min(w, 300);
    R = diag([(covar_vec(1) * w)^2 * ones(3,1); covar_vec(2)^2 * ones(3,1); covar_vec(3)^2 * ones(3,1)]);
    
    % A priori covariance and Kalman gain
    L = P * H' / (H * P * H' + R);
    
    % Predicted measurements 
    z_hat = [R_b2i' * [0; 0; constantsASTRA.g];
             x_est(10:12);
             R_b2i' * constantsASTRA.mag];
    
    % Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(15) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z(1:9) - z_hat);
    dx = dx + L * residual;
end
if sum(lastZ(10:15) - z(10:15)) ~=0

    % Measurement matrix
    H = zeros(6,15);
    H(1:3, 4:6) = eye(3);
    H(4:6, 7:9) = eye(3);

    % Measurement Covariance Matrix
    gps_pos_covar = 0.2;
    gps_vel_covar = 0.2;
    R = diag([gps_pos_covar^2 * ones(3,1); gps_vel_covar^2 * ones(3,1)]);

    % A priori covariance and Kalman gain
    L = P * H' / (H * P * H' + R);

    % Predicted measurements 
    z_hat = [x_est(4:6);
             x_est(7:9)];

    % Kalman Gain Weighting based on predicted acceleration
    ILH = (eye(15) - L * H);
    P = ILH * P * ILH' + L * R * L';
    residual = (z(10:15) - z_hat);
    inn = L * residual;
    dx = dx + inn;
end

% Update full-state estimates
q0 = sqrt(abs(1 - x_est(1:3)'*x_est(1:3)));
q = [q0; x_est(1:3)];
dq = [1; dx(1:3) / 2];

q_nom = quatmultiply(q', dq');
q_nom = q_nom / norm(q_nom); 
x_est(1:3) = q_nom(2:4)';
x_est(4:15) = x_est(4:15) + dx(4:15);
dx(1:3) = diag(P(1:3, 1:3));
lastZ = z;
end