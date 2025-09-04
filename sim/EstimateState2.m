function X_est = EstimateState2(Y, X_hat, U, t)
    
%% Single Kalman Filter Estimation (Extended Kalman Filter)
persistent P

%% Load System Dynamics
% X_crit and U_crit are linearization points, for an EKF arquitechture we
% linearize around the last estimated state and the current inputs.
% Exception for T < 1 sec to avoid linearization around invalid points. 

% Calculate Jacobians
if t < 4
    X_crit = zeros(15,1);
    U_crit = [0; 0; 1.5*9.8; 0];
else
    X_crit = X_hat;
    U_crit = U;
end

% Jacobian function (specific to system dynamics, comprised of elementary
% operations), obtained thanks to MATLAB.
J_x = AUG_JacobianX(X_crit, U_crit);
J_h = JacobianH(X_crit);

% Observability Matrix for IMU Update Step in EKF (all states observable
% through dual integration + magnetometer)
H1 = [eye(3) zeros(3, 12); 
     zeros(3,3) eye(3) zeros(3,9);  
     J_h; zeros(3,9) eye(3) zeros(3)];

% Covariance matrix for Kalman filter, initialized as an [n x n] matrix,
% where n is the number of states (~13 for ASTRA v2), Consider deflating
% P0, or inflating.
if isempty(P)
    P = eye(15);
    X_hat = [zeros(12,1); 0.12; 0.12; 0.12];
end

% Discretize the dynamics using zero order hold, standard operation.
% timestep h to be chosen.
h = 1/500;
A_d = expm(J_x*h);

% Measurements (Y is the measurements we get back, in this case it is
% backed out by multiplying the observability matrix by the full state, but
% since we're *not* supposed to know thw full state, we only use the
% measurements. The observability matrix for ASTRA v2 depends on which
% sensors we have, dimensions [m x n] where m is the number of
% measurements. In our case, probably angular rates, positions (from GPS),
% and accelerations.

% Standard deviations of every state measurement (obtained experimentally
% or just estimated)
Rvec = [0.2 0.2 0.2 0.2 0.2 0.2 1/4 1/4 1/4 0.2 0.2 0.2 0.01*ones(1,3)];

% Measurement Noise Covariancce Matrix
% (' operator indicates transpose, diag creates a diagonal matrix with the
% vector elements across the diagonal)
R =  diag((H1*Rvec').^2);

% Process Noise Covariance Matrix (obtained experimentally, size [n x n])
% Q = 0.0005 * eye(12);
Q = diag([1e-4 1e-4 1e-4 1e-4 1e-4 1e-4 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6*ones(1,3)]);

% Conditioning check for inverted matrix (look for the documentation of the
% RCOND function in MATLAB for implementation)
isWCond = (rcond(H1*P*H1' + R*R') > 1e-9);
if isWCond == 0
    % Set estimated state to -999 to signal ill-conditioned matrix and flag
    % the filter.
    X_est = ones(15,1)*-999;
else
    % Calculates the Kalman Gain (inv is the matrix inverse.)
    L = A_d*P*H1'*inv(H1*P*H1' + R*R');
    
    % Prediction and Estimation (uses Euler integration to integrate the
    % state derivative obtained from the nonlinear plant dynamics).
    X_est = X_hat + AUG_plantfcn(X_hat, U)*h + L*(Y - H1*X_hat);

    % Updates the covariance matrix.
    P = A_d*P*A_d' + Q*Q' - L*H1*P*A_d';
end

