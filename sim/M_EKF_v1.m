function X_est = M_EKF_v1(Y, X_hat, U, t)
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

%Prediction
X_est = X_hat + AUG_plantfcn(X_hat, U)*h;

%Predicted measurements 
