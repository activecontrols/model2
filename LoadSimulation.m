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
covar_vec = [accel_proc_cov; gyro_cov; mag_proc_cov];
IMU_Rate = 1000;     %Hz

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

%% Generate LQR Controller for Simulation
% Brysons Rule for Q and R.
a_weights = ones(12,1);
b_weights = ones(4,1);
a_weights = a_weights / norm(a_weights);
b_weights = b_weights / norm(b_weights);

max_x = [3, 3, 0.5, 1000, 1000, 1000, 1, 1, 0.4, pi/8, pi/8, 2];
% max_x = [0.5, 0.5, 0.5, 1000, 1000, 1000, 1, 1, 0.4, 1000, 1000, 2];
max_u = [pi/18, pi/18, 6, 0.4];

Q = eye(size(linSys.A,1)) .* a_weights ./ max_x.^2;
% R = eye(size(linSys.B,2)) .* b_weights ./ max_u.^2;
R = diag([260, 260, 4, 10]);
% R = diag([60, 60, 3, 10]);

%[K, ~, ~] = lqr(linSys.A, linSys.B, Q, R);
K = [0.017107038138553	-0.000000000000000	-0.000000000000000	-0.000000000000000	-0.000004194996784	-0.000000000000000	-0.000000000000000	-0.000085570908421	-0.000000000000000	0.044409791827320	-0.000000000000000	-0.000000000000000;
-0.000000000000000	0.017145814117362	-0.000000000000000	0.000004194996784	0.000000000000000	0.000000000000000	0.000085667825890	0.000000000000000	0.000000000000000	0.000000000000000	0.044604848326624	-0.000000000000000;
0.000000000000007	-0.000000000000001	0.000000000000000	-0.000000000000000	-0.000000000000000	0.000043130805076	-0.000000000000000	-0.000000000000000	2.537760653503638	0.000000000000002	-0.000000000000001	-0.000000000000000;
-0.000000000000003	0.000000000000000	0.000031622776602	-0.000000000000000	0.000000000000000	0.000000000000000	-0.000000000000000	0.000000000000000	-0.000000000000002	-0.000000000000000	-0.000000000000000	2.623358221415387];

%% Checkpoints and HoldTimes for Trajectory
Checkpoints = [0, 0, 0,  3,  3, 0, 0, 0;
               0, 0, 3,  3,  0, 0, 0, 0;
               0, 3, 3,  3,  3, 3, 0, 0];
HoldTimeReqs = [4, 3, 3, 3, 3, 3, 0, 0.2];

