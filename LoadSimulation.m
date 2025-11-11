%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file loads in all the constants and parameters for the Simulink into
% workspace. Please always run this file before running a full-scale
% simulation if you've made any changes to trajectory, controls, filtering,
% or others.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize parameters and clear functions
% Initial conditions for state
% YOU CHANGED THE ACTUATORS DUMMY, IF IT DOESNT WORK IT'S CUZ OF THAT DONT
% GO INSANE THX -Pablo
clear;
clear ref_generator3;
clear inputfcn3;
clear EstimateStateFCN;
clear SensorSimulation;
clear GPS_Sim;

addpath('.\Parameters');
addpath('.\Filtering');
addpath('.\Filtering\EMA Filter');
addpath('.\Filtering\Kalman Filter');
addpath('.\Filtering\ANF');
addpath('.\Simulation');
addpath('.\Trajectory');
addpath('.\Actuators');
addpath('.\Controls');
addpath('.\Sensors');
addpath('.\Plotting');
constants_port;
constantsASTRA = constructConstants;
constantsASTRA.Q = p2.Q;
constantsASTRA.R = p2.obsv_cov_mat;
covar_vec = [accel_proc_cov; gyro_cov; mag_proc_cov];

%%
x0 = zeros(15,1);
u0 = [0; 0; constantsASTRA.g * constantsASTRA.m; 0];

%% Generate nominal dynamics function
% Documentation for the math available on Confluence.
[x, u2, x_dot] = EoMGenerator(constantsASTRA, 2);
[linSys, disLinSys] = dynamics(x, u2, x_dot, constantsASTRA);
matlabFunction(x_dot, 'File', './Simulation/nominalDynamics.m', 'Vars', [{x}, {u2}]);
linSys.A = linSys.A(1:12,1:12);
linSys.B = linSys.B(1:12,:);
constantsASTRA.mag = [cos(pi/6); 0; -sin(pi/6)];
ASTRAv2 = Simulink.Bus.createObject(constantsASTRA);
% Simulink.Bus.createObject(linSys);

%% Generate LQR Controller for Simulation
% Brysons Rule for Q and R.
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [5, 5, 0.08, 1000, 1000, 1000, 0.55, 0.55, 2, 1, 1, 2];
max_u = [pi/45, pi/45, 6, 0.5];

Q = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
R = eye(size(linSys.B,2)) .* b_weights ./ max_u.^2;

[K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);

%% Checkpoints and HoldTimes for Trajectory
Checkpoints = [0, 0, 0,  3,  3, 0, 0, 0;
               0, 0, 3,  3,  0, 0, 0, 0;
               0, 3, 3,  3,  3, 3, 0, 0];
HoldTimeReqs = [4, 3, 3, 3, 3, 3, 0, 0.2];
