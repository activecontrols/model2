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
clear GPS_Sim;
clear DigitalNF;

addpath('.\cpp');
addpath('.\Parameters');
addpath('.\Filtering');
addpath('.\Filtering\EMA Filter');
addpath('.\Filtering\Kalman Filter');
addpath('.\Filtering\ANF');
addpath('.\Simulation');
addpath('.\Simulation\Disturbances\');
addpath('.\Simulation\Helper\');
addpath('.\Simulation\Stability\');
addpath('.\Simulation\Vehicle Motion\');
addpath('.\Trajectory');
addpath('.\Actuators');
addpath('.\Controls');
addpath('.\Sensors');
addpath('.\Plotting');
constants_port;
constantsASTRA = constructConstants;
constantsASTRA.Q = p2.Q;
constantsASTRA.R = p2.obsv_cov_mat;
constantsASTRA.MaxT = constantsASTRA.g * 1.697;
covar_vec = [accel_proc_cov; gyro_cov; mag_proc_cov];
IMU_Rate = 1000;     %Hz

%%
x0 = [1; zeros(15,1)];
u0 = [0; 0; constantsASTRA.g * constantsASTRA.m; 0];

%% Generate nominal dynamics function
% Documentation for the math available on Confluence.
% MODE == 2 on EoMGen produces clean Dynamics, Mode 1 produces perturbed.
[x, u2, x_dot, x_ref, u_ref, e_x, e_u, e_x_dot] = EoMGenerator(constantsASTRA, 2);
matlabFunction(x_dot, 'File', './Simulation/Vehicle Motion/nominalDynamics.m', 'Vars', [{x}, {u2}]);
%matlabFunction(e_x_dot, 'File', './Simulation/Vehicle Motion/nominalErrDynamics.m', 'Vars', [{e_x}, {e_u}, {x_ref}, {u_ref}])
[linSys, disLinSys] = dynamics(x, u2, x_dot, constantsASTRA);
linSysErr = errorDynamics(e_x, e_u, x_ref, u_ref, e_x_dot, constantsASTRA);

[x, u2, x_dot, x_ref, u_ref, e_x, e_u, e_x_dot] = EoMGenerator(constantsASTRA, 1);
matlabFunction(x_dot, 'File', './Simulation/Vehicle Motion/disturbedDynamics.m', 'Vars', [{x}, {u2}]);
%matlabFunction(e_x_dot, 'File', './Simulation/Vehicle Motion/disturbedErrDynamics.m', 'Vars', [{e_x}, {e_u}, {x_ref}, {u_ref}])

linSys.A = linSys.A(1:12,1:12);
linSys.B = linSys.B(1:12,:);

constantsASTRA.mag = [cos(pi/6); 0; -sin(pi/6)];
magDistMatrix = eye(3) + 0.02 * randn(3);
magBias = 0.05 * ones(1,3);
gyroBias = 0.005 * ones(1,3);
accelBias = [0.09, 0.09, 0.09];

%% Attitude Controller Generation
[K_Att, ~] = Controller2_Gen(constantsASTRA);
constantsASTRA.K_Att = K_Att;
ASTRAv2 = Simulink.Bus.createObject(constantsASTRA);

%% Generate LQR Controller for Simulation
% Brysons Rule for Q and R.
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [3, 3, 0.3, 1000, 1000, 1000, 0.5, 0.5, 0.4, 1000, 1000, 0.5];
% max_x = [0.5, 0.5, 0.5, 1000, 1000, 1000, 1, 1, 0.4, 1000, 1000, 2];
max_u = [pi/24, pi/24, 6, 2];

Q = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
R = eye(size(linSys.B,2)) .* b_weights ./ max_u.^2;
% R = diag([260, 260, 0.05, 0.2]);
% R = diag([60, 60, 3, 10]);

[K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);

%% Checkpoints and HoldTimes for Trajectory
Checkpoints =  [0, 0, 0,  3,  3, 0, 0, 0;
                0, 0, 3,  3,  0, 0, 0, 0;
                0, 3, 3,  3,  3, 3, 0, 0];
HoldTimeReqs = [7, 5, 3, 3, 3, 3, 0, 0.2];
% Checkpoints =  [0, 5, 0;
%                 0, 10, 0;
%                 0, 50, 0];
% HoldTimeReqs = [5, 10, 5];

% Disturbances (1 for on, 0 for off)
distMode = 1; 
dt_SIM = 1/1000;


