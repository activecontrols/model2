function z = SensorSimulation(x, x_dot, constantsASTRA, covar_vec, t)

% Simulates magnetometer, accelerometer, and gyros given the true state and
% a covariance vector. No biases for now.

% Update rates
IMU_Rate = 1000;     %Hz
GPS_Rate = 15;      %Hz
z = zeros(15,1);

persistent lastGPS lastIMU lastZ bias_vec_G bias_vecA

if isempty(lastGPS)
    lastGPS = 0;  % Update last GPS timestamp
    lastIMU = 0;  % Update last IMU timestamp
    lastZ = [0; 0; 9.81; zeros(3,1); 1; 0; 0; zeros(6,1)];
    bias_vec_G = 0.05 * ones(3,1);
    bias_vecA = 0; % 0.02 * ones(3,1);
end

% Extract Quaternion
q0 = sqrt(abs(1 - x(1:3)'*x(1:3)));
q = [q0; x(1:3)];
R_b2i = quatRot(q)';
bias_vec_G = bias_vec_G + 5e-5 * randn(3,1);
%bias_vecA = bias_vecA + 5e-5 * randn(3,1);

% Fake IMU measurements
if t - lastIMU > 1 / IMU_Rate
    z(1:3) = R_b2i' * (x_dot(7:9) + [0; 0; constantsASTRA.g]) + bias_vecA + covar_vec(1) * randn;
    z(4:6) = x(10:12) + covar_vec(2) * randn + bias_vec_G;
    z(7:9) = R_b2i' * constantsASTRA.mag + covar_vec(3) * randn;
    lastIMU = t;
else
    z(1:9) = lastZ(1:9);
end

% Fake GPS measurements
if t - lastGPS > 1 / GPS_Rate
    gps_pos_covar = 0.02;
    gps_vel_covar = 0.15;
    z(10:12) = x(4:6) + gps_pos_covar * randn;
    z(13:15) = x(7:9) + gps_vel_covar * randn;
    lastGPS = t;
else
    z(10:15) = lastZ(10:15);
end

lastZ = z;