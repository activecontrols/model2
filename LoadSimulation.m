%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file loads in all the constants and parameters for the Simulink into
% workspace. Please always run this file before running a full-scale
% simulation if you've made any changes to trajectory, controls, filtering,
% or others.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize parameters and clear functions
% Initial conditions for state
clear;
clear ref_generator3;
clear inputfcn3;
clear EstimateStateFCN;
clear SensorSimulation;

addpath('.\Parameters');
addpath('.\State Estimation\EMA Filter');
addpath('.\State Estimation\Kalman Filter');
addpath('.\SimFiles');
addpath('.\Trajectory');
addpath('.\Actuators\');
constants;
constantsASTRA = constructConstants;
Simulink.Bus.createObject(constantsASTRA);
covar_vec = [accel_proc_cov; gyro_cov; mag_proc_cov];

%%
x0 = zeros(15,1);
u0 = [0; 0; constantsASTRA.g * constantsASTRA.m; 0];

%% Generate nominal dynamics function
% Documentation for the math available on Confluence.
[x, u2, x_dot, ~] = EoMGenerator(constantsASTRA, 2);
[linSys, disLinSys] = dynamics(x, u2, x_dot, constantsASTRA);
matlabFunction(x_dot, 'File', './SimFiles/nominalDynamics.m', 'Vars', [{x}, {u2}]);
linSys.A = linSys.A(1:12,1:12);
linSys.B = linSys.B(1:12,:);

%% Generate LQR Controller for Simulation
% Brysons Rule for Q and R.
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [2, 2, 0.08, 1000, 1000, 1000, 0.8, 0.8, 2, 2, 2, 10];
max_u = [pi/30, pi/30, 6, 0.5];

Q = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
R = eye(size(linSys.B,2)) .* b_weights ./ max_u.^2;

[K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);
