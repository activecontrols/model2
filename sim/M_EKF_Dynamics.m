%% Generation of Error State Dynamics for a Multiplicative Quaternion Kalman Filter ( M-EKF ) Implementation for ASTRA v2

syms r1 r2 r3           % Positions
syms v1 v2 v3           % Velocities 
syms q0 q1 q2 q3        % Vector quaternion
syms a1 a2 a3           % Small angle errors

syms w1 w2 w3           % Angular rates
syms ac1 ac2 ac3        % Accelerometer
syms m1 m2 m3           % Magnetometer

syms b1 b2 b3           % Gyro biases
syms m l g rTB mag_ref  % system constants


%IMU Measurement vectors
accel = [ac1; ac2; ac3];
gyro = [w1; w2; w3];
magneto = [m1; m2; m3];

% Position error vector
r = [r1; r2; r3];

% Velocity error vector
v = [v1; v2; v3];

% Quaternion error vector
q = [q0; q1; q2; q3];

% Small angle error parameter vector 
alpha = [a1; a2; a3];

% Gyro bias error vector
b = [b1; b2; b3];

% Error state vector
dx = [alpha; r; v; b];

%DCM from Earth to Body
C_BI = quatRot(q);

%DCM from Body to Earth
C_IB = C_BI.';

% State transition matrix (cont.)
F = [-zetaCross(gyro) zeros(3, 6) -eye(3);
     -C_IB * zetaCross(accel) zeros(3, 9);
     zeros(3) eye(3) zeros(3, 6);
     zeros(3, 12)];

% Full IMU Measurement
IMU = [accel, gyro, magneto];

% Generate MATLAB function
matlabFunction(F, 'File', './sim/lib/StateTransMat.m', 'Vars', [{IMU}, {q}]);