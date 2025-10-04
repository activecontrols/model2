%%Initializing constants
%Sensor Bias:
gyro_bias = [-15.267 -9.8963 -2.37]*(pi/180); %Measured in dps, converted to rad/s
accel_bias = [0.00171 -0.66371 0.01735]; %m/s^2
mag_bias = [0.035 0.045 -0.015]; %G
dt = 40/10000; %assuming a 200Hz filter + sensor poll/update rate

%Covariance Defines (p = variable to store constants and pass into our matlab function):
p.est_cov = 1.0; %How much we trust obsv vs. measurements initially

%Gyro (rad/s) - estimated from datasheet & testing
gyro_bias_cov = 0.1;
gyro_cov = 0.01;

%Accel (m/s^2) - estimated from datasheet & testing
accel_obs_cov = .5;
accel_proc_cov = 0.1; %How much we expect to deviate between updates
accel_bias_cov = 0.1;

%Mag (G/s) - estimated from datasheet & testing

mag_proc_cov = 0.01;
mag_bias_cov = 0.1;
mag_obs_cov = 0.1;

%Matrix Defines
p.obsv_cov_mat = eye(6);
p.obsv_cov(1:3,1:3) = accel_obs_cov * eye(3);
p.obsv_cov(4:6,4:6) = mag_obs_cov * eye(3);

p.gyro_cov_mat = gyro_cov*eye(3);
p.gyro_bias_cov_mat = gyro_bias_cov*eye(3);
p.accel_cov_mat = accel_proc_cov*eye(3);
p.accel_bias_cov_mat = accel_bias_cov*eye(3);
p.mag_cov_mat = mag_proc_cov*eye(3);
p.mag_bias_cov_mat = mag_bias_cov*eye(3);

%Process noise covariance, statically defined for fixed timestep
p.Q = zeros(15);
p.Q(1:3, 1:3) = p.gyro_cov_mat*dt + p.gyro_bias_cov_mat*(dt^3)/3.0;
p.Q(1:3, 13:15) = -p.gyro_bias_cov_mat*(dt^2)/2.0;
p.Q(7:9, 4:6) = p.accel_cov_mat*dt;
p.Q(7:9, 7:9) = p.accel_cov_mat*(dt^2)/2.0;
p.Q(4:6, 4:6) = p.accel_cov_mat*(dt^2)/2.0;
p.Q(4:6, 7:9) = p.accel_cov_mat*(dt^3)/3.0;
p.Q(13:15, 1:3) = -p.gyro_bias_cov_mat*(dt^2)/2.0;
p.Q(13:15, 13:15) = p.gyro_bias_cov_mat*dt;

% Hand-tuning factors
p.Q(1, 1) = p.Q(1, 1) * 0.001;
p.Q(2:3, 2:3) = p.Q(2:3, 2:3) * 0.025;